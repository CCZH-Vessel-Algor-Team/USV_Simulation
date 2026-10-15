"""Model description helpers shared by localization and standalone simulation."""

import os
import re


def convert_model_uri_to_package_uri(urdf_content: str) -> str:
    """Convert known Gazebo model:// URIs to RViz-compatible package:// URIs."""
    from ament_index_python.packages import get_package_share_directory

    model_to_pkg = {}

    try:
        wamv_desc_share = get_package_share_directory('wamv_description')
        wamv_models_dir = os.path.join(wamv_desc_share, 'models')
        if os.path.isdir(wamv_models_dir):
            for name in os.listdir(wamv_models_dir):
                if os.path.isdir(os.path.join(wamv_models_dir, name)):
                    model_to_pkg[name] = f'package://wamv_description/models/{name}/'
    except Exception:
        pass

    try:
        usv_sim_share = get_package_share_directory('usv_sim_full')
        usv_models_dir = os.path.join(usv_sim_share, 'description', 'models')
        if os.path.isdir(usv_models_dir):
            for name in os.listdir(usv_models_dir):
                if os.path.isdir(os.path.join(usv_models_dir, name)):
                    model_to_pkg[name] = f'package://usv_sim_full/description/models/{name}/'
    except Exception:
        pass

    def replace_model_uri(match):
        model_name = match.group(1)
        sub_path = match.group(2)
        if model_name in model_to_pkg:
            return model_to_pkg[model_name] + sub_path
        return match.group(0)

    return re.sub(r'model://([^/]+)/(.+)', replace_model_uri, urdf_content)
