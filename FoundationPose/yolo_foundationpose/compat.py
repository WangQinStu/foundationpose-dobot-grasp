import torch


def patch_torch_load_for_old_ultralytics():
  """兼容旧版 Ultralytics 在 PyTorch 2.6+ 下加载完整 checkpoint 的行为。"""
  if getattr(torch.load, '_foundationpose_yolo_patch', False):
    return

  torch_load = torch.load

  def torch_load_compat(*args, **kwargs):
    if 'weights_only' not in kwargs:
      kwargs['weights_only'] = False
    return torch_load(*args, **kwargs)

  torch_load_compat._foundationpose_yolo_patch = True
  torch.load = torch_load_compat

