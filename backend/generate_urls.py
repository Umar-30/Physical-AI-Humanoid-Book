#!/usr/bin/env python3
"""
Generate all Docusaurus URLs from docs/ folder
"""
import os
from pathlib import Path

# Base URL for the documentation
BASE_URL = "https://umar-30.github.io/Physical-AI-Humanoid-Book"

# Path to docs folder (relative to backend/)
DOCS_PATH = Path("..") / "docs"

def generate_urls():
    """Generate all documentation URLs"""
    urls = []

    # Walk through docs directory
    for root, dirs, files in os.walk(DOCS_PATH):
        for file in files:
            if file.endswith(('.md', '.mdx')) and file != 'DOCUSAURUS_RESEARCH_SUMMARY.md':
                # Get relative path from docs/
                file_path = Path(root) / file
                rel_path = file_path.relative_to(DOCS_PATH)

                # Convert to URL path (remove .md/.mdx extension)
                url_path = str(rel_path).replace('\\', '/').replace('.mdx', '').replace('.md', '')

                # Handle index files
                if url_path.endswith('/index'):
                    url_path = url_path[:-6]  # Remove '/index'

                # Create full URL
                if url_path:
                    url = f"{BASE_URL}/docs/{url_path}"
                    urls.append(url)

    return sorted(set(urls))

if __name__ == "__main__":
    urls = generate_urls()

    print(f"Found {len(urls)} documentation pages\n")
    print("=" * 60)

    # Save to file
    output_file = "urls.txt"
    with open(output_file, 'w') as f:
        for url in urls:
            f.write(url + '\n')

    print(f"URLs saved to: {output_file}")
    print(f"\nFirst 10 URLs:")
    for i, url in enumerate(urls[:10], 1):
        print(f"{i:2d}. {url}")

    if len(urls) > 10:
        print(f"\n... and {len(urls) - 10} more")
