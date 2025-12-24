
#!/usr/bin/env python3
"""
Web Crawler for Docusaurus Site
Discovers all pages by following links
"""
import requests
from bs4 import BeautifulSoup
from urllib.parse import urljoin, urlparse
from typing import Set
import time

def is_same_domain(url1: str, url2: str) -> bool:
    """Check if two URLs are from the same domain"""
    return urlparse(url1).netloc == urlparse(url2).netloc

def is_valid_page(url: str, base_url: str) -> bool:
    """Check if URL is a valid page to crawl"""
    parsed = urlparse(url)

    # Must be same domain
    if not is_same_domain(url, base_url):
        return False

    # Ignore fragments
    if '#' in url:
        return False

    # Ignore file downloads
    ignore_extensions = ['.pdf', '.zip', '.png', '.jpg', '.jpeg', '.gif', '.svg', '.ico']
    if any(url.lower().endswith(ext) for ext in ignore_extensions):
        return False

    return True

def crawl_site(base_url: str, max_pages: int = 100) -> Set[str]:
    """
    Crawl a website starting from base_url and discover all pages

    Args:
        base_url: Starting URL
        max_pages: Maximum pages to crawl (safety limit)

    Returns:
        Set of discovered URLs
    """
    visited = set()
    to_visit = {base_url}
    discovered_urls = set()

    print(f"Starting crawl from: {base_url}")
    print(f"Max pages: {max_pages}\n")

    while to_visit and len(visited) < max_pages:
        url = to_visit.pop()

        if url in visited:
            continue

        print(f"[{len(visited)+1}/{max_pages}] Crawling: {url}")
        visited.add(url)
        discovered_urls.add(url)

        try:
            response = requests.get(url, timeout=10)
            response.raise_for_status()

            soup = BeautifulSoup(response.content, 'html.parser')

            # Find all links
            for link in soup.find_all('a', href=True):
                href = link['href']
                absolute_url = urljoin(url, href)

                # Remove trailing slash for consistency
                absolute_url = absolute_url.rstrip('/')

                # Check if valid and not visited
                if is_valid_page(absolute_url, base_url) and absolute_url not in visited:
                    to_visit.add(absolute_url)

            # Small delay to be respectful
            time.sleep(0.5)

        except Exception as e:
            print(f"  Error: {str(e)}")
            continue

    print(f"\n{'=' * 60}")
    print(f"Crawl complete!")
    print(f"Total pages discovered: {len(discovered_urls)}")
    print(f"{'=' * 60}\n")

    return discovered_urls

if __name__ == "__main__":
    base_url = "https://umar-30.github.io/Physical-AI-Humanoid-Book"

    urls = crawl_site(base_url, max_pages=200)

    # Save to file
    output_file = "discovered_urls.txt"
    with open(output_file, 'w') as f:
        for url in sorted(urls):
            f.write(url + '\n')

    print(f"URLs saved to: {output_file}")
    print("\nDiscovered URLs:")
    for i, url in enumerate(sorted(urls), 1):
        print(f"{i:3d}. {url}")
