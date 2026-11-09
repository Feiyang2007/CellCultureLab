"""Read a bounded part of one pinned public wheel through the configured proxy.

No filesystem writes, direct-network fallback, background transfer or arbitrary
URL support. The caller verifies the assembled wheel SHA before extraction.
"""
import argparse
import base64
import json
import urllib.request
import requests

URL = "https://files.pythonhosted.org/packages/03/56/f5d8e2ac2df979044cb18c8862de7be9c2247921f8f7b06d45d3daf99c15/mujoco-3.3.0-cp311-cp311-win_amd64.whl"
SHA256 = "436d863747fea327393a46f04d7aeaea6acf27b961b54dbdde32a823e90355eb"
SIZE = 4860052


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--offset", type=int, required=True)
    a = p.parse_args()
    if not 0 <= a.offset < SIZE or a.offset % 196608:
        p.error("Offset must be a valid fixed-size wheel chunk boundary")
    proxy = urllib.request.getproxies().get("https")
    if not proxy:
        raise RuntimeError("Configured HTTPS proxy required; direct fallback disabled")
    end = min(SIZE, a.offset+196608)-1
    session = requests.Session()
    session.trust_env = False
    session.proxies = {"https":proxy}
    response = session.get(URL, headers={"Range":f"bytes={a.offset}-{end}"}, timeout=30)
    response.raise_for_status()
    if response.status_code == 206:
        data = response.content
        if response.headers.get("Content-Range") != f"bytes {a.offset}-{end}/{SIZE}":
            raise RuntimeError("Unexpected content range")
    elif response.status_code == 200 and len(response.content) == SIZE:
        data = response.content[a.offset:end+1]
    else:
        raise RuntimeError("Unexpected artifact response")
    if len(data) != end-a.offset+1:
        raise RuntimeError("Incomplete wheel chunk")
    print(json.dumps(dict(offset=a.offset,size=len(data),expected_sha256=SHA256,
        configured_https_proxy_used=True,base64=base64.b64encode(data).decode("ascii"))),flush=True)


if __name__ == "__main__":
    main()
