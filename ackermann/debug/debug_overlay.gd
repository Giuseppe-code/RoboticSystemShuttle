extends Node

class NullStats:
	func add_property(_target: Object, _property: String, _display: String = "") -> void:
		pass

class NullDraw:
	func add_vector(_target: Object, _property: String, _scale: float, _width: float, _color: Color) -> void:
		pass

var stats = NullStats.new()
var draw = NullDraw.new()
