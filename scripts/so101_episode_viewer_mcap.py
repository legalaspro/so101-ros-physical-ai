#!/usr/bin/env python3
from __future__ import annotations

import argparse
import threading
import time
import uuid
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterator

import gradio as gr
from gradio_rerun import Rerun
import rerun as rr
import rerun.blueprint as rrb

# ==============================================================================
# CONFIGURATION & STYLING
# ==============================================================================

APP_ID = "so101_episode_browser"

CSS = """
#episode_list_wrap {
  height: 750px;
  overflow-y: auto;
  border: 1px solid var(--border-color-primary);
  border-radius: 8px;
  padding: 8px;
}
#episode_dataset {
  width: 100%;
}
/* Ensure the list doesn't grow infinitely */
#episode_dataset .dataset,
#episode_dataset .wrap,
#episode_dataset .table-wrap {
  max-height: 720px;
  overflow-y: auto;
}
"""

@dataclass(frozen=True)
class Episode:
    name: str
    folder: Path
    mcaps: list[Path]

# ==============================================================================
# BLUEPRINT DEFINITION
# ==============================================================================

def build_so101_blueprint() -> rrb.Blueprint:
    return rrb.Blueprint(
        rrb.Vertical(
            # Top Row: Images
            rrb.Horizontal(
                rrb.Spatial2DView(
                    origin="/follower/image_raw",
                    name="Follower Camera"
                ),
                rrb.Spatial2DView(
                    origin="/static_camera/image_raw",
                    name="Static Camera"
                ),
                column_shares=[1, 1]
            ),
            # Bottom Row: Time Series
            rrb.Horizontal(
                rrb.TimeSeriesView(
                    origin="/follower/joint_states",
                    name="Joint States"
                ),
                rrb.TimeSeriesView(
                    origin="/follower/forward_controller/commands",
                    name="Controller Commands"
                ),
                column_shares=[1, 1]
            ),
            row_shares=[2, 1] # Images get 2/3rds height, graphs get 1/3rd
        ),
        collapse_panels=True,
    )

# ==============================================================================
# FILE HANDLING
# ==============================================================================

def _human_bytes(n: int) -> str:
    for u in ["B", "KB", "MB", "GB"]:
        if n < 1024.0:
            return f"{n:.1f} {u}" if u != "B" else f"{n} {u}"
        n /= 1024.0
    return f"{n:.1f} TB"

def index_episodes(root: Path) -> list[Episode]:
    if not root.exists():
        return []

    mcaps = sorted(root.rglob("*.mcap"))
    by_folder: dict[Path, list[Path]] = {}

    for p in mcaps:
        # Group by the immediate parent folder
        by_folder.setdefault(p.parent, []).append(p)

    episodes: list[Episode] = []
    for folder, files in sorted(by_folder.items()):
        # Name is relative to root
        rel_name = str(folder.relative_to(root)) if folder != root else folder.name
        episodes.append(Episode(name=rel_name, folder=folder, mcaps=sorted(files)))

    return episodes

def make_labels(episodes: list[Episode]) -> list[str]:
    labels = []
    for ep in episodes:
        total_size = sum(f.stat().st_size for f in ep.mcaps)
        # Find newest modification time
        ts = 0.0
        if ep.mcaps:
            ts = max(f.stat().st_mtime for f in ep.mcaps)

        time_str = time.strftime('%Y-%m-%d %H:%M', time.localtime(ts))

        label = (
            f"📁 {ep.name}\n"
            f"{len(ep.mcaps)} MCAP(s) · {_human_bytes(total_size)}\n"
            f"Last mod: {time_str}"
        )
        labels.append(label)
    return labels



# ==============================================================================
# STREAMING LOGIC
# ==============================================================================

def stream_episode(
    dataset_index: int,
    episodes_state: list[dict],
    current_recording_id: str
) -> Iterator[tuple[Any, Any, str]]:

    # 1. Validation
    if dataset_index is None or not episodes_state:
        yield gr.skip(), "No episodes found.", current_recording_id
        return

    if dataset_index < 0 or dataset_index >= len(episodes_state):
        yield gr.skip(), "Selection Error", current_recording_id
        return

    ep_data = episodes_state[dataset_index]
    mcaps = [Path(p) for p in ep_data["mcaps"]]
    ep_name = ep_data["name"]

    # 2. Setup Rerun Streaming
    # Create a unique ID so the viewer knows this is a new session
    new_recording_id = str(uuid.uuid4())

    # Initialize the stream
    rec = rr.RecordingStream(application_id=APP_ID, recording_id=new_recording_id)
    stream = rec.binary_stream()

    # Send the custom Blueprint immediately
    bp = build_so101_blueprint()
    rec.send_blueprint(bp)

    # 3. Threaded Loading
    # We parse MCAP in a background thread so we can yield bytes to the browser immediately
    done_event = threading.Event()

    def loader_worker():
        try:
            for mcap_file in mcaps:
                rr.log_file_from_path(str(mcap_file), recording=rec)
        except Exception as e:
            print(f"Error loading MCAP {mcap_file}: {e}")
        finally:
            # Important: Disconnect closes the stream properly
            rr.disconnect(recording=rec)
            done_event.set()

    t = threading.Thread(target=loader_worker, daemon=True)
    t.start()

    # 4. Generator Loop
    # Yield status update
    yield gr.skip(), f"Loading `{ep_name}`...", new_recording_id

    try:
        while not done_event.is_set():
            # Read all available data from the pipe
            chunk = stream.read()
            if chunk:
                yield chunk, gr.skip(), new_recording_id
            else:
                # Small sleep to prevent tight loop if buffer is empty but thread is running
                time.sleep(0.01)

        # Flush remaining data
        while True:
            chunk = stream.read()
            if not chunk:
                break
            yield chunk, gr.skip(), new_recording_id

        yield gr.skip(), f"Streaming `{ep_name}` active", new_recording_id

    except GeneratorExit:
        # User closed tab or clicked something else
        pass


# ==============================================================================
# MAIN APP
# ==============================================================================

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--episodes_root", type=Path, default=Path("."), help="Folder containing episode subfolders")
    args = parser.parse_args()

    root = args.episodes_root.expanduser().resolve()

    # Initial Indexing
    episodes = index_episodes(root)
    # Convert to simple dicts for Gradio State
    ep_state_init = [
        {"name": e.name, "folder": str(e.folder), "mcaps": [str(p) for p in e.mcaps]}
        for e in episodes
    ]
    labels_init = make_labels(episodes)
    # Dummy samples for the dataset component (it needs a list of lists)
    samples_init = [[""] for _ in labels_init]

    with gr.Blocks(theme=gr.themes.Soft(), css=CSS, title="SO-101 Data Browser") as demo:

        # --- State ---
        episodes_state = gr.State(ep_state_init)
        recording_id = gr.State("")

        # --- Layout ---
        with gr.Row():
            # Left Sidebar
            with gr.Column(scale=1, min_width=300):
                gr.Markdown("# 🤖 SO-101 Episode Browser")

                with gr.Column(elem_id="episode_list_wrap"):
                    # Hidden component required by Dataset logic
                    _hidden = gr.Textbox(visible=False)

                    episode_list = gr.Dataset(
                        label="Episodes",
                        components=[_hidden],
                        samples=samples_init,
                        sample_labels=labels_init,
                        type="index", # Returns integer index on click
                        elem_id="episode_dataset",
                        samples_per_page=100
                    )

                status_md = gr.Markdown("Ready.")

            # Right Viewer
            with gr.Column(scale=4):
                viewer = Rerun(
                    streaming=True,
                    height=800,
                    panel_states={
                        "blueprint": "hidden",
                        "selection": "hidden",
                        "time": "collapsed"
                    },
                )

        # --- Callbacks ---

        # Click & Stream
        episode_list.click(
            fn=stream_episode,
            inputs=[episode_list, episodes_state, recording_id],
            outputs=[viewer, status_md, recording_id],
            concurrency_limit=1 # Ensure only one stream happens at a time
        )

    print(f"Index complete. Found {len(episodes)} episodes.")
    demo.queue().launch(server_name="0.0.0.0", inbrowser=True)

if __name__ == "__main__":
    main()