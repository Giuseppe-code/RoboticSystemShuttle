extends Node3D

@export var vehicle_path: NodePath = ^"../DemoCar"
@export var carried_offset := Vector3(0.0, 1.25, 0.15)
@export var delivered_position := Vector3(-2.5, 0.32, 0.0)
@export var delivered_position_c := Vector3(0.0, 0.32, -7.5)
@export var package_size := Vector3(0.55, 0.55, 0.55)

var vehicle: Node3D
var carried_package: MeshInstance3D
var delivered_package: MeshInstance3D
var saw_loaded_payload := false


func _ready() -> void:
	vehicle = get_node_or_null(vehicle_path) as Node3D
	DDS.subscribe("PayloadMass")
	DDS.subscribe("MissionState")
	DDS.subscribe("CargoColorCode")
	DDS.subscribe("UnloadZoneCode")
	carried_package = _make_package(_package_color(1.0))
	delivered_package = _make_package(_package_color(1.0))
	add_child(carried_package)
	add_child(delivered_package)
	carried_package.visible = false
	delivered_package.visible = false
	delivered_package.global_position = delivered_position


func _process(_delta: float) -> void:
	var payload_mass = DDS.read("PayloadMass")
	var mission_state = DDS.read("MissionState")
	var cargo_color_code = DDS.read("CargoColorCode")
	var unload_zone_code = DDS.read("UnloadZoneCode")
	var payload := 0.0 if payload_mass == null else float(payload_mass)
	var color_code := 1.0 if cargo_color_code == null else float(cargo_color_code)
	var zone_code := 1.0 if unload_zone_code == null else float(unload_zone_code)
	var package_color := _package_color(color_code)
	_set_package_color(carried_package, package_color)
	_set_package_color(delivered_package, package_color)

	if payload > 0.01:
		saw_loaded_payload = true
		carried_package.visible = true
		delivered_package.visible = false
		if vehicle != null:
			carried_package.global_position = vehicle.global_position + carried_offset
			carried_package.global_rotation = vehicle.global_rotation
		return

	carried_package.visible = false
	if saw_loaded_payload and _is_unload_or_done(mission_state):
		delivered_package.visible = true
		delivered_package.global_position = _delivered_position(zone_code)


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


func _package_color(color_code: float) -> Color:
	if int(color_code) == 2:
		return Color(0.9, 0.05, 0.04, 1.0)
	return Color(0.0, 0.66, 0.85, 1.0)


func _delivered_position(zone_code: float) -> Vector3:
	if int(zone_code) == 3:
		return delivered_position_c
	return delivered_position


func _set_package_color(package: MeshInstance3D, color: Color) -> void:
	if package == null or package.mesh == null:
		return
	var material := package.mesh.material as StandardMaterial3D
	if material != null:
		material.albedo_color = color


func _is_unload_or_done(mission_state) -> bool:
	if mission_state == null:
		return false
	return int(float(mission_state)) >= 6
