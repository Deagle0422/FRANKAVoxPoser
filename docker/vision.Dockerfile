FROM pytorch/pytorch:2.5.1-cuda12.4-cudnn9-runtime

ARG SAM2_REF=2b90b9f5ceec907a1c18123530e92e794ad901a4
ARG USER_UID=1000
ARG USER_GID=1000
ARG USERNAME=vision

ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    PYTHONPATH=/workspace/voxposer/src \
    HF_HOME=/models/huggingface \
    SAM2_BUILD_CUDA=0

RUN apt-get update && apt-get install -y --no-install-recommends \
      ca-certificates \
      git \
    && rm -rf /var/lib/apt/lists/*

RUN pip install --no-cache-dir \
      numpy==2.0.2 \
      pillow==11.3.0 \
      transformers==4.57.3 \
      accelerate==1.10.1 \
      safetensors==0.6.2 \
    && git clone https://github.com/facebookresearch/sam2.git /opt/sam2 \
    && git -C /opt/sam2 checkout "${SAM2_REF}" \
    && pip install --no-cache-dir --no-deps /opt/sam2

RUN groupadd --gid "${USER_GID}" "${USERNAME}" \
    && useradd --uid "${USER_UID}" --gid "${USER_GID}" --create-home "${USERNAME}"

WORKDIR /workspace/voxposer
COPY --chown=${USER_UID}:${USER_GID} voxposer /workspace/voxposer
RUN mkdir -p /models/huggingface \
    && chown -R "${USER_UID}:${USER_GID}" /models

USER ${USERNAME}
CMD ["python", "-m", "perception.vision_service"]
