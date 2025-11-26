import logging


def configure_toolbox_logging(level=logging.INFO) -> logging.Logger:
    """
    Configures toolbox level logging
    :param level: logging level
    """
    logger = logging.getLogger("commonroad_control")
    handler = logging.StreamHandler()
    handler.setLevel(level)
    formatter = logging.Formatter("%(name)s: %(message)s")
    handler.setFormatter(formatter)
    logger.addHandler(handler)
    logger.setLevel(level)
    return logger
