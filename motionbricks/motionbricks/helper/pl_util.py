from hydra.utils import instantiate
from omegaconf import DictConfig


def load_motion_rep(conf: DictConfig):
    skeleton = instantiate(conf.skeleton)
    motion_rep = instantiate(conf.motion_rep, fps=conf.fps, skeleton=skeleton)
    return motion_rep
