FROM python:3.9-slim-bookworm

ARG USER_UID=1000
ARG USER_GID=1000
ARG USERNAME=voxposer

ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    VIRTUAL_ENV=/opt/venv \
    PATH=/opt/venv/bin:$PATH \
    PYTHONPATH=/workspace/voxposer/src

RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential \
      ca-certificates \
      libgl1 \
      libglib2.0-0 \
      libgomp1 \
    && rm -rf /var/lib/apt/lists/*

RUN python -m venv /opt/venv \
    && pip install --no-cache-dir --upgrade pip==25.3

COPY voxposer/requirements.docker.txt /tmp/requirements.txt
RUN pip install --no-cache-dir -r /tmp/requirements.txt

RUN groupadd --gid "${USER_GID}" "${USERNAME}" \
    && useradd --uid "${USER_UID}" --gid "${USER_GID}" --create-home "${USERNAME}"

WORKDIR /workspace/voxposer
COPY --chown=${USER_UID}:${USER_GID} voxposer /workspace/voxposer
RUN mkdir -p /workspace/outputs \
    && chown -R "${USER_UID}:${USER_GID}" /workspace

USER ${USERNAME}
CMD ["bash"]

