#!/usr/bin/env python3
# Copyright 2026 Dmitri Manajev
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Iterable, Optional

from huggingface_hub import DatasetCard, HfApi, metadata_update


def _read_text(path: str | None) -> str | None:
    if not path:
        return None
    p = Path(path).expanduser()
    if not p.exists():
        raise SystemExit(f"File not found: {p}")
    return p.read_text(encoding="utf-8")


def _dedup_preserve_order(items: Iterable[str]) -> list[str]:
    seen: set[str] = set()
    out: list[str] = []
    for x in items:
        if x not in seen:
            seen.add(x)
            out.append(x)
    return out


def _merge_tags(existing: list[str] | None, new: list[str]) -> list[str]:
    existing = existing or []
    return _dedup_preserve_order([*existing, *new])


def _fetch_repo_text(api: HfApi, repo_id: str, path_in_repo: str) -> Optional[str]:
    """Download a small text file from a dataset repo. Returns None if missing."""
    try:
        data = api.hf_hub_download(
            repo_id=repo_id, repo_type="dataset", filename=path_in_repo
        )
        return Path(data).read_text(encoding="utf-8")
    except Exception:
        return None


def _build_structure_section(info_json_text: str) -> str:
    # Pretty-print json with stable formatting
    try:
        obj = json.loads(info_json_text)
        pretty = json.dumps(obj, indent=4, ensure_ascii=False)
    except Exception:
        # If parsing fails, just show raw
        pretty = info_json_text.strip()

    return (
        "## Dataset Structure\n\n"
        "[meta/info.json](meta/info.json):\n"
        "```json\n"
        f"{pretty}\n"
        "```\n"
    )


def _build_citation_section(bibtex: str) -> str:
    bibtex = bibtex.strip()
    return f"## Citation\n\n**BibTeX:**\n\n```bibtex\n{bibtex}\n```\n"


def main() -> None:
    ap = argparse.ArgumentParser(
        description="Replace HF dataset README body from a base markdown file, then append structure (meta/info.json) and citation."
    )
    ap.add_argument(
        "--repo-id", required=True, help="e.g. legalaspro/so101-ros-physical-ai-test"
    )
    ap.add_argument(
        "--token",
        default=None,
        help="HF token (optional; otherwise uses cached login/env).",
    )

    # Metadata
    ap.add_argument(
        "--tag",
        action="append",
        default=[],
        help="Repeatable tags. Example: --tag ros2 --tag so101",
    )
    ap.add_argument("--license", default=None, help="HF license id, e.g. apache-2.0")

    # Replace body base
    ap.add_argument(
        "--replace-body",
        required=True,
        help="Path to base markdown (your intro/description). Script will append structure + citation automatically.",
    )

    # Optional: citation source & upload
    ap.add_argument(
        "--citation-bib",
        default=None,
        help="Path to local CITATION.bib to upload and embed.",
    )
    ap.add_argument(
        "--embed-citation",
        action="store_true",
        default=False,
        help="Embed BibTeX into README (in addition to uploading CITATION.bib if provided).",
    )

    # Optional: structure embedding
    ap.add_argument(
        "--embed-structure",
        action="store_true",
        default=True,
        help="Embed meta/info.json in README (default: on).",
    )
    ap.add_argument(
        "--structure-details",
        action="store_true",
        default=True,
        help="Wrap the JSON in a <details> block to reduce scrolling (default: on).",
    )

    # Commit messages
    ap.add_argument(
        "--commit-message",
        default="Update dataset card",
        help="Commit message for README/metadata changes.",
    )
    ap.add_argument(
        "--citation-commit-message",
        default="Add/Update CITATION.bib",
        help="Commit message for citation upload.",
    )

    args = ap.parse_args()
    repo_id = args.repo_id

    api = HfApi(token=args.token)

    # Load current card (to merge tags safely)
    card = DatasetCard.load(repo_id, repo_type="dataset", token=args.token)
    current_tags = None
    try:
        current_tags = card.data.to_dict().get("tags")
    except Exception:
        current_tags = None

    # ---- 1) Update YAML metadata (merge tags + license) ----
    meta: dict = {}
    if args.tag:
        meta["tags"] = _merge_tags(current_tags, args.tag)
    if args.license:
        meta["license"] = args.license

    if meta:
        metadata_update(
            repo_id=repo_id,
            repo_type="dataset",
            metadata=meta,
            overwrite=True,  # safe because tags are merged and license is intentional
            token=args.token,
            commit_message=args.commit_message,
        )

    # ---- 2) Build README body from base + structure + citation ----
    base_md = _read_text(args.replace_body).strip()
    parts: list[str] = [base_md]

    if args.embed_structure:
        info_text = _fetch_repo_text(api, repo_id, "meta/info.json")
        if info_text:
            section = _build_structure_section(info_text)
            # Simpler: just wrap the full section
            if args.structure_details:
                section = (
                    "## Dataset Structure\n\n"
                    "<details>\n<summary>Show meta/info.json</summary>\n\n"
                    + _build_structure_section(info_text).split(
                        "## Dataset Structure\n\n", 1
                    )[1]
                    + "\n</details>\n"
                )
            parts.append(section)
        else:
            # still provide a link even if file missing
            parts.append("## Dataset Structure\n\n[meta/info.json](meta/info.json)\n")

    # --- Citation ---
    bibtex_text = None
    if args.citation_bib:
        bib_path = Path(args.citation_bib).expanduser()
        if not bib_path.exists():
            raise SystemExit(f"--citation-bib not found: {bib_path}")
        bibtex_text = bib_path.read_text(encoding="utf-8").strip()

        api.upload_file(
            path_or_fileobj=str(bib_path),
            path_in_repo="CITATION.bib",
            repo_id=repo_id,
            repo_type="dataset",
            commit_message=args.citation_commit_message,
        )

    if bibtex_text:
        parts.append(f"## Citation\n\n**BibTeX:**\n\n```bibtex\n{bibtex_text}\n```\n")
    else:
        parts.append("## Citation\n\nSee `CITATION.bib` in this repo.\n")

    final_body = "\n\n".join(p.strip() for p in parts if p and p.strip()) + "\n"

    # Push updated README body
    card = DatasetCard.load(repo_id, repo_type="dataset", token=args.token)
    card.text = "\n" + final_body
    card.push_to_hub(
        repo_id=repo_id,
        repo_type="dataset",
        token=args.token,
        commit_message=args.commit_message,
    )


if __name__ == "__main__":
    main()
