#!/usr/bin/env python3

from PyQt5.QtWidgets import (QTreeWidget, QTreeWidgetItem, QStyledItemDelegate,
                             QLineEdit, QDoubleSpinBox, QSpinBox, QCheckBox, QWidget, QHBoxLayout)
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtGui import QColor

class Columns:
    """Constants for Tree Widget Columns"""
    NAME = 0
    VALUE = 1
    TYPE = 2

def get_param_category(type_name: str) -> str:
    """
    Normalize parameter type names to basic categories.
    
    Returns:
        'int', 'float', 'bool', or 'str'
    """
    type_name = type_name.lower()
    if type_name in ['int', 'integer', 'int64']:
        return 'int'
    elif type_name in ['float', 'double']:
        return 'float'
    elif type_name in ['bool', 'boolean']:
        return 'bool'
    return 'str'

class ParameterTreeWidget(QTreeWidget):
    """Custom tree widget for displaying and editing ROS2 parameters"""
    
    parameter_changed = pyqtSignal(str, object)  # (param_name, new_value)
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setColumnCount(3)
        self.setHeaderLabels(["Parameter", "Value", "Type"])
        
        self.setColumnWidth(Columns.NAME, 300)
        self.setColumnWidth(Columns.VALUE, 200)
        self.setColumnWidth(Columns.TYPE, 100)
        
        self.setItemDelegate(ParameterItemDelegate(self))
        self.itemChanged.connect(self.on_item_changed)
        
        self.parameters = {}
        self.item_to_param = {}
        self.updating = False
        
    def set_parameters(self, node_name: str, parameters: dict):
        """Set parameters for a specific node"""
        self.clear()
        self.parameters = parameters
        self.item_to_param.clear()
        
        self._build_tree(node_name, parameters)
        self.expandAll()
        
    def set_yaml_parameters(self, parameters: dict):
        """Set parameters from YAML file (flat structure)"""
        self.clear()
        self.parameters = parameters
        self.item_to_param.clear()
        
        self._build_tree_from_paths(parameters)
        self.expandAll()
        
    def _build_tree(self, node_name: str, parameters: dict):
        """Build tree structure from node parameters"""
        for param_name, param_info in parameters.items():
            self._add_parameter_item(None, param_name, param_info)
            
    def _build_tree_from_paths(self, parameters: dict):
        """Build hierarchical tree from flat parameter paths"""
        tree_structure = {}
        
        for param_path, value in parameters.items():
            parts = param_path.split('.')
            current = tree_structure
            
            for i, part in enumerate(parts[:-1]):
                if part not in current:
                    current[part] = {}
                current = current[part]
            
            # Last part is the actual parameter
            current[parts[-1]] = value
            
        self._add_tree_structure(None, tree_structure, [])
        
    def _add_tree_structure(self, parent, structure, path):
        """Recursively add tree structure"""
        for key, value in structure.items():
            current_path = path + [key]
            
            if isinstance(value, dict):
                item = QTreeWidgetItem(parent or self)
                item.setText(Columns.NAME, key)
                item.setForeground(Columns.NAME, QColor(100, 150, 255))
                self._add_tree_structure(item, value, current_path)
            else:
                param_path = '.'.join(current_path)
                param_info = {'value': value, 'type': type(value).__name__}
                self._add_parameter_item(parent, key, param_info, param_path)
                
    def _add_parameter_item(self, parent, param_name: str, param_info: dict, full_path: str | None = None):
        """Add a parameter item to the tree"""
        item = QTreeWidgetItem(parent or self)
        item.setText(Columns.NAME, param_name)
        
        value = param_info.get('value')
        param_type = param_info.get('type', type(value).__name__)
        
        item.setText(Columns.VALUE, str(value))
        item.setFlags(item.flags() | Qt.ItemFlag.ItemIsEditable)
        item.setText(Columns.TYPE, param_type)
        
        path = full_path if full_path else param_name
        self.item_to_param[item] = path
        
        # Color code by category
        category = get_param_category(param_type)
        color_map = {
            'int': QColor(100, 200, 100),
            'float': QColor(100, 150, 255),
            'bool': QColor(255, 150, 100),
            'str': QColor(200, 200, 100)
        }
        
        if category in color_map:
            item.setForeground(Columns.TYPE, color_map[category])

    def update_parameter_values(self, parameters: dict):
        """Update parameter values without rebuilding tree"""
        self.updating = True
        for param_name, param_info in parameters.items():
            for item, path in self.item_to_param.items():
                if path == param_name:
                    new_value = param_info.get('value')
                    current_value = item.text(Columns.VALUE)
                    if str(new_value) != current_value:
                        item.setText(Columns.VALUE, str(new_value))
                    break
        self.updating = False
    
    def itemFromIndex(self, index):
        """Get QTreeWidgetItem from model index"""
        if not index.isValid():
            return None
            
        item = None
        parent_index = index.parent()
        
        if not parent_index.isValid():
            item = self.topLevelItem(index.row())
        else:
            parent_item = self.itemFromIndex(parent_index)
            if parent_item:
                item = parent_item.child(index.row())
                
        return item
        
    def on_item_changed(self, item: QTreeWidgetItem, column: int):
        """Handle item change (user edited value)"""
        # Only react to changes in the Value column
        if self.updating or column != Columns.VALUE:
            return
            
        param_path = self.item_to_param.get(item)
        if not param_path:
            return
            
        new_value_str = item.text(Columns.VALUE)
        param_type = item.text(Columns.TYPE)
        category = get_param_category(param_type)
        
        try:
            if category == 'int':
                new_value = int(new_value_str)
            elif category == 'float':
                new_value = float(new_value_str)
            elif category == 'bool':
                new_value = new_value_str.lower() in ['true', '1', 'yes']
            else:
                new_value = new_value_str
                
            self.parameter_changed.emit(param_path, new_value)
            
        except ValueError:
            # Revert on error
            if param_path in self.parameters:
                item.setText(Columns.VALUE, str(self.parameters[param_path].get('value')))


class ParameterItemDelegate(QStyledItemDelegate):
    """Custom delegate for parameter editing"""
    
    def createEditor(self, parent, option, index):
        """Create appropriate editor based on parameter type"""
        if index.column() != Columns.VALUE:
            return super().createEditor(parent, option, index)
            
        tree_widget = self.parent()
        if not isinstance(tree_widget, ParameterTreeWidget):
            return super().createEditor(parent, option, index)

        item = tree_widget.itemFromIndex(index)
        if item is None:
            return super().createEditor(parent, option, index)
            
        param_type = item.text(Columns.TYPE)
        category = get_param_category(param_type)
        
        if category == 'int':
            editor = QSpinBox(parent)
            editor.setRange(-2147483648, 2147483647)
            return editor
        elif category == 'float':
            editor = QDoubleSpinBox(parent)
            editor.setRange(-1e6, 1e6)
            editor.setDecimals(4)
            return editor
        elif category == 'bool':
            widget = QWidget(parent)
            layout = QHBoxLayout(widget)
            layout.setContentsMargins(0, 0, 0, 0)
            checkbox = QCheckBox(widget)
            layout.addWidget(checkbox)
            return widget
        else:
            return QLineEdit(parent)
            
    def setEditorData(self, editor, index):
        """Set initial data in editor"""
        if index.column() != Columns.VALUE:
            return super().setEditorData(editor, index)
            
        value = index.data(Qt.ItemDataRole.DisplayRole)
        
        # Editors are already created with specific types, so we can use isinstance checks safely
        if isinstance(editor, QSpinBox):
            editor.setValue(int(value))
        elif isinstance(editor, QDoubleSpinBox):
            editor.setValue(float(value))
        elif isinstance(editor, QWidget):
            checkbox = editor.findChild(QCheckBox)
            if checkbox:
                checkbox.setChecked(value.lower() in ['true', '1', 'yes'])
        elif isinstance(editor, QLineEdit):
            editor.setText(value)
            
    def setModelData(self, editor, model, index):
        """Save editor data back to model"""
        if index.column() != Columns.VALUE or model is None:
            return super().setModelData(editor, model, index)
            
        if isinstance(editor, QSpinBox):
            model.setData(index, str(editor.value()), Qt.ItemDataRole.EditRole)
        elif isinstance(editor, QDoubleSpinBox):
            model.setData(index, str(editor.value()), Qt.ItemDataRole.EditRole)
        elif isinstance(editor, QWidget):
            checkbox = editor.findChild(QCheckBox)
            if checkbox:
                model.setData(index, str(checkbox.isChecked()), Qt.ItemDataRole.EditRole)
        elif isinstance(editor, QLineEdit):
            model.setData(index, editor.text(), Qt.ItemDataRole.EditRole)