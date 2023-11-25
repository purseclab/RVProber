This is a fork of the flight_review repository's main branch.

## How to download all logs from PX4 Review Website.

- `download_db.json` has been added under `flight_review\app`.
    - The log downloader uses an internal API which is broken.
    - So I captured the public API which fetches data to the website and obtained the `px4_review_website_response.json` file, and made it into `download_db.json`
- Invoke `download_logs.py`