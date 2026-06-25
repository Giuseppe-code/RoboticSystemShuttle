extends Node3D

@export var vehicle_path: NodePath = ^"../DemoCar"
@export var carried_offset := Vector3(0.0, 1.25, 0.15)
@export var delivered_position := Vector3(-2.5, 0.32, 0.0)
@export var package_size := Vector3(0.55, 0.55, 0.55)

var vehicle: Node3D
var carried_package: MeshInstance3D
var delivered_package: MeshInstance3D
var saw_loaded_payload := false


func _ready() -> void:
	vehicle = get_node_or_null(vehicle_path) as Node3D
	DDS.subscribe("PayloadMass")
	DDS.subscribe("CargoPhase")
	carried_package = _make_package(Color(0.05, 0.22, 0.95, 1.0))
	delivered_package = _make_package(Color(0.05, 0.22, 0.95, 1.0))
	add_child(carried_package)
	add_child(delivered_package)
	carried_package.visible = false
	delivered_package.visible = false
	delivered_package.global_position = delivered_position


func _process(_delta: float) -> void:
	var payload_mass = DDS.read("PayloadMass")
	var cargo_phase = DDS.read("CargoPhase")
	var payload := 0.0 if payload_mass == null else float(payload_mass)
	var phase := 0.0 if cargo_phase == null else float(cargo_phase)

	if payload > 0.01:
		saw_loaded_payload = true
		carried_package.visible = true
		delivered_package.visible = false
		if vehicle != null:
			carried_package.global_position = vehicle.global_position + carried_offset
			carried_package.global_rotation = vehicle.global_rotation
		return

	carried_package.visible = false
	if saw_loaded_payload and phase >= 3.0:
		delivered_package.visible = true
		delivered_package.global_position = delivered_position


func _make_package(color: Color) -> MeshInstance3D:
	var mesh_instance := MeshInstance3D.new()
	var mesh := BoxMesh.new()
	mesh.size = package_size
	var material := StandardMaterial3D.new()
	material.albedo_color = color
	material.roughness = 0.55
	mesh.material = material
	mesh_instance.mesh = mesh
	mesh_instance.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_ON
	return mesh_instance
