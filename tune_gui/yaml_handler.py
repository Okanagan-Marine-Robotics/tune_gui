#!/usr/bin/env python3

import yaml
from typing import Dict, Any
from pathlib import Path


class YAMLHandler:
    """Handler for loading, editing, and saving params.yaml files"""
    
    def __init__(self):
        self.yaml_path = None
        self.yaml_data = {}
        self.flat_params = {}
        
    def load_yaml(self, file_path: str):
        """Load parameters from YAML file"""
        self.yaml_path = Path(file_path)
        
        with open(self.yaml_path, 'r') as f:
            self.yaml_data = yaml.safe_load(f)
            
        if not self.yaml_data:
            self.yaml_data = {}
            
        self.flat_params = self._flatten_dict(self.yaml_data)
        
    def save_yaml(self, file_path: str | None = None):
        """Save current parameters to YAML file"""
        if file_path:
            self.yaml_path = Path(file_path)
            
        if not self.yaml_path:
            raise ValueError("No file path specified")
            
        nested = self._unflatten_dict(self.flat_params)
        
        with open(self.yaml_path, 'w') as f:
            yaml.dump(nested, f, default_flow_style=False, sort_keys=False)
            
    def get_all_parameters(self) -> Dict[str, Any]:
        """Get all parameters as flat dictionary"""
        return self.flat_params.copy()
        
    def get_parameter(self, param_path: str) -> Any:
        """Get a specific parameter value by path"""
        return self.flat_params.get(param_path)
        
    def update_parameter(self, param_path: str, value: Any):
        """Update a parameter value"""
        if param_path not in self.flat_params:
            raise KeyError(f"Parameter {param_path} not found")
            
        self.flat_params[param_path] = value
        
    def _flatten_dict(self, d: Dict, parent_key: str = '', sep: str = '.') -> Dict:
        """Flatten nested dictionary to dotted keys
        
        Example:
            {'a': {'b': {'c': 1}}} -> {'a.b.c': 1}
        """
        items = []
        for k, v in d.items():
            new_key = f"{parent_key}{sep}{k}" if parent_key else k
            
            if isinstance(v, dict):
                items.extend(self._flatten_dict(v, new_key, sep=sep).items())
            else:
                items.append((new_key, v))
                
        return dict(items)
        
    def _unflatten_dict(self, d: Dict, sep: str = '.') -> Dict:
        """Unflatten dotted keys to nested dictionary
        
        Example:
            {'a.b.c': 1} -> {'a': {'b': {'c': 1}}}
        """
        result = {}
        
        for key, value in d.items():
            parts = key.split(sep)
            current = result
            
            for part in parts[:-1]:
                if part not in current:
                    current[part] = {}
                current = current[part]
                
            current[parts[-1]] = value
            
        return result
        
    def get_parameters_for_node(self, node_name: str) -> Dict[str, Any]:
        """Get all parameters for a specific node"""
        prefix = f"{node_name}."
        node_params = {}
        
        for key, value in self.flat_params.items():
            if key.startswith(prefix):
                param_name = key[len(prefix):]
                if param_name.startswith('ros__parameters.'):
                    param_name = param_name[len('ros__parameters.'):]
                node_params[param_name] = value
                
        return node_params