"""OWLv2 box detection followed by SAM 2 image segmentation.

Heavy ML imports are intentionally lazy: this module is used by the optional
vision container, while the main VoxPoser planner remains a small Python 3.9
environment.
"""

from contextlib import nullcontext

import numpy as np


class OpenVocabularySAM2:
    """Return the SAM 2 mask for OWLv2's highest-confidence matching box."""

    def __init__(
        self,
        detector_model="google/owlv2-base-patch16-ensemble",
        sam2_model="facebook/sam2.1-hiera-tiny",
        detection_threshold=0.15,
        device="auto",
    ):
        import torch
        from sam2.sam2_image_predictor import SAM2ImagePredictor
        from transformers import AutoModelForZeroShotObjectDetection, AutoProcessor

        if device == "auto":
            device = "cuda" if torch.cuda.is_available() else "cpu"
        self.torch = torch
        self.device = torch.device(device)
        self.detection_threshold = float(detection_threshold)
        self.processor = AutoProcessor.from_pretrained(detector_model)
        self.detector = AutoModelForZeroShotObjectDetection.from_pretrained(
            detector_model
        ).to(self.device)
        self.detector.eval()
        self.predictor = SAM2ImagePredictor.from_pretrained(
            sam2_model, device=str(self.device)
        )

    def __call__(self, query: str, rgb_image: np.ndarray) -> np.ndarray:
        image = np.asarray(rgb_image, dtype=np.uint8)
        height, width = image.shape[:2]
        inputs = self.processor(
            text=[[str(query)]], images=image, return_tensors="pt"
        ).to(self.device)
        with self.torch.inference_mode():
            outputs = self.detector(**inputs)
        result = self.processor.post_process_grounded_object_detection(
            outputs=outputs,
            target_sizes=[(height, width)],
            threshold=self.detection_threshold,
            text_labels=[[str(query)]],
        )[0]
        if len(result["boxes"]) == 0:
            raise LookupError(
                f"OWLv2 found no object matching {query!r} at threshold "
                f"{self.detection_threshold}"
            )
        best = int(self.torch.argmax(result["scores"]).item())
        box = result["boxes"][best].detach().cpu().numpy()

        autocast = (
            self.torch.autocast("cuda", dtype=self.torch.bfloat16)
            if self.device.type == "cuda"
            else nullcontext()
        )
        with self.torch.inference_mode(), autocast:
            self.predictor.set_image(image)
            masks, scores, _ = self.predictor.predict(
                box=box, multimask_output=False
            )
        if len(masks) == 0:
            raise RuntimeError(f"SAM 2 returned no mask for {query!r}")
        best_mask = int(np.argmax(scores))
        return np.asarray(masks[best_mask], dtype=bool)
