import argparse
import asyncio
from .gateway import run


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    parser.add_argument("--api-url", default="http://127.0.0.1:8000")
    parser.add_argument("--state-dir", required=True)
    args = parser.parse_args()
    asyncio.run(run(args.config, args.api_url, args.state_dir))


if __name__ == "__main__":
    main()
