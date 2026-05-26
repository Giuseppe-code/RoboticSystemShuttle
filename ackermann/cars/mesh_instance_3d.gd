extends MeshInstance3D

func _ready() -> void:
	var static_body = StaticBody3D.new()
	add_child(static_body)
	
	var collision = CollisionShape3D.new()
	var shape = BoxShape3D.new()
	shape.size = mesh.get_aabb().size
	
	collision.shape = shape
	collision.position = mesh.get_aabb().get_center()
	collision.rotation = rotation  # Copia TUTTA la rotazione dalla mesh!
	
	static_body.add_child(collision)
	print("Rotazione copiata!")
