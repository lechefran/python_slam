FROM python:3.12-slim-bookworm
WORKDIR /app
COPY pyproject.toml README.md LICENSE ./
COPY slam.py frame.py geometry.py point.py dmap.py display.py tracking_diagnostics.py ./
RUN python -m pip install --no-cache-dir .
ENTRYPOINT ["python-slam", "--headless"]
