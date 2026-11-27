"""Build the configured segmentation adapter."""

from .color_segmenter import ColorRuleSegmenter
from .remote_segmenter import RemoteSegmenter


def build_segmenter(config):
    """Create a segmenter from the ``rgbd.segmentation`` configuration."""

    segmenter_type = str(config.get("type", "color_rules")).lower()
    objects = config.get("objects")
    if not objects:
        raise ValueError("segmentation.objects must define at least one object")
    if segmenter_type == "color_rules":
        return ColorRuleSegmenter(objects)
    if segmenter_type in {"remote_sam2", "remote_open_vocabulary_sam2"}:
        service = config.get("service", {})
        return RemoteSegmenter(
            objects,
            endpoint=service.get("endpoint", "http://127.0.0.1:8770"),
            timeout=service.get("timeout_sec", 60.0),
        )
    raise ValueError(
        f"Unsupported segmentation.type={segmenter_type!r}; expected "
        "'color_rules' or 'remote_sam2'"
    )
