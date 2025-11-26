import logging
import os
import time
from pathlib import Path
from typing import Union

from PIL import Image

logger = logging.getLogger(__name__)


def make_gif(
    path_to_img_dir: Union[Path, str],
    scenario_name: str,
    num_imgs: int,
    duration: float = 0.1,
    abort_img_threshold: int = 200,
) -> None:
    if (
        not os.path.exists(path_to_img_dir)
        or not os.path.isdir(path_to_img_dir)
        or not os.path.isabs(path_to_img_dir)
    ):
        logger.error(
            f"image dir {path_to_img_dir} must exist, be a directory and be absolute"
        )
        raise FileNotFoundError(
            f"image dir {path_to_img_dir} must exist, be a directory and be absolute"
        )

    # get all files in dir
    imgs = sorted(
        [el for el in os.listdir(path_to_img_dir) if ".png" in el],
        key=lambda x: int(x.split(".")[0].split("_")[-1]),
    )

    logger.debug("creating gif")

    # poll until all imgs ware saved
    cnt = 0
    while len(imgs) != num_imgs and cnt < 50:
        imgs = sorted(
            [el for el in os.listdir(path_to_img_dir) if ".png" in el],
            key=lambda x: int(x.split(".")[0].split("_")[-1]),
        )
        time.sleep(0.2)
        cnt += 1

    if cnt == abort_img_threshold:
        logger.error("Could not find all expected imgs")
        raise ValueError("Could not find all expected imgs")

    imgs_pil = [Image.open(os.path.join(path_to_img_dir, img)) for img in imgs]
    output_path = os.path.join(path_to_img_dir, scenario_name + ".gif")

    imgs_pil[0].save(
        output_path,
        save_all=True,
        append_images=imgs_pil[1:],
        duration=duration,
        loop=0,
    )
