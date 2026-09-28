##a node that renders the pathfinding system of a Complex
@tool class_name ComplexRender extends Node

const POINT_MESH: SphereMesh = preload("uid://cgg5ugiapdkxg") ##the mesh rendered for the points on the navigation grid
const CONNECTION_MATERIAL: StandardMaterial3D = preload("uid://yt7042shagqe") ##the material rendered onto a path line on the navigation grid
const COLOR_DISABLED: Color = Color(0.0, 0.0, 0.0, 1.0)   ##color of points on the navigation grid that are disabled
const COLOR_DEFAULT:  Color = Color(0.0, 0.0, 0.0, 0.122) ##color of points on the navigation grid by default
const COLOR_PATH:     Color = Color(1.0, 0.0, 0.0, 1.000) ##color of points on the navigation grid that are also on a path line
const PATH_MIN_RADIUS: float = 0.025 ##the smallest radius that can be drawn for a path

@export var complex: Complex: ##the Complex to render
	set(v): #setting disconnects the refresh signal to the render function from the old complex and reconnects it to the new one
		if complex != v:
			if complex and complex.refreshed.is_connected(render): complex.refreshed.disconnect(render)
			complex = v
			if v and !v.refreshed.is_connected(render): v.refreshed.connect(render)

@export_tool_button("Reset") var reset: Callable = _reset ##calls the reset function, clearing the rendering nodes

var _mmp: MultiMeshInstance3D: ##the mesh displaying the points on the AStar3D grid
	get(): #ensures that there is always a valid instance- setting up and adding the mesh to the Complex
		if !is_instance_valid(_mmp):
			_mmp = MultiMeshInstance3D.new()
			var mm: MultiMesh = MultiMesh.new()
			mm.transform_format = MultiMesh.TRANSFORM_3D
			mm.use_colors = true
			mm.mesh = POINT_MESH
			_mmp.multimesh = mm
			add_child(_mmp)
		return _mmp
var _mc: MeshInstance3D: ##the mesh displaying path lines
	get(): #ensures that there is always a valid instance- setting up and adding the mesh to the Complex
		if !is_instance_valid(_mc):
			_mc = MeshInstance3D.new()
			_mc.material_override = CONNECTION_MATERIAL
			add_child(_mc)
		return _mc

##clears the rendering nodes
func _reset() -> void:
	if is_instance_valid(_mmp): _mmp.queue_free()
	if is_instance_valid(_mc): _mc.queue_free()

##renders the AStar3D grid, disabled points, and currently tracked pathes for debug visualization
func render() -> void:
	var path: Dictionary[int, bool] = {} #all points that are on a path- again Dictionarys are faster than Arrays
	var vts: PackedVector3Array = PackedVector3Array() #prisim triangles that are turned into a mesh at the end
	for a: Node3D in complex.paths: #for each agent, 
		var p: Complex.Path = complex.paths[a] #get the path
		if p.ids.is_empty(): continue #skip empty ones
		var astar: AStar3D = complex.astar_from_mode(p.mode) #select astar based on path mode
		#create the prism from the agent position to the target point (also adding it to be
		#rendered differenty), correctly displaying shortcuts
		var r: float = maxf(p.radius, PATH_MIN_RADIUS)
		if astar.has_point(p.ids[p.idx]):
			path[p.ids[p.idx]] = true
			_prism(vts, a.global_position, astar.get_point_position(p.ids[p.idx]), r)
		for i: int in range(p.idx, p.ids.size()): #for each point in the path from the current target,
			path[p.ids[i]] = true #add the point to be rendered differently in the MultiMesh pass
			if i < p.ids.size() - 1 and astar.has_point(p.ids[i]) and astar.has_point(p.ids[i + 1]): #if astar has both points (current and next one),
				_prism(vts, astar.get_point_position(p.ids[i]), astar.get_point_position(p.ids[i + 1]), r) #create the prisim between the points
	#using the verticies, create the immediate mesh and add it to the node to render it
	var im: ImmediateMesh = ImmediateMesh.new()
	if !vts.is_empty():
		im.surface_begin(Mesh.PRIMITIVE_TRIANGLES)
		for v: Vector3 in vts: im.surface_add_vertex(v)
		im.surface_end()
	_mc.mesh = im
	#get the point ids from the AStar3D and the MultiMesh from the node
	var ids: PackedInt64Array = complex.points
	var mm: MultiMesh = _mmp.multimesh
	mm.instance_count = ids.size() #number of instances, points rendered, is the number of points in the AStar3D
	#iterate thru a range of numbers. range() is not used because that makes a new Array, which is
	#expensive. the actual IDs arent iterated through either because the MultiMesh needs an index for
	#each of the instances, points, that it renders.
	for idx: int in ids.size():
		var i: int = ids[idx] #get the point ID
		var astar: AStar3D = complex.astar_from_id(i) #get the astar based on the point ID
		#set the instance's position at index idx to the position on the AStar3D grid. because there's
		#no set_instance_position() function, we make a Transform3D using a default basis (positioned
		#at 0, no rotation, and scale of x1) and move it's origin (position) to the position of i.
		mm.set_instance_transform(idx, Transform3D(Basis(), astar.get_point_position(i)))
		#sets the color of the points based on of their disabled, part of a path, or just idle
		mm.set_instance_color(idx, COLOR_DISABLED if astar.is_point_disabled(i)
			else (COLOR_PATH if path.has(i) else COLOR_DEFAULT))

##creates a cylinder with only 3 sides stretching from a to b with radius r
func _prism(vts: PackedVector3Array, a: Vector3, b: Vector3, r: float) -> void:
	#gets the direction of the segment, returns if too small
	var d: Vector3 = b - a
	if is_zero_approx(d.length()): return
	d = d.normalized() #normalize it so it'a actually a direction and no length
	#create a perpendicular frame- the up is the global up unless the direction is pointing
	#up, where forward is then used. e1 and e2 use cross each time to create the other
	#2 axises to create this frame
	var up: Vector3 = Vector3.UP if absf(d.dot(Vector3.UP)) < 0.99 else Vector3.FORWARD
	var e1: Vector3 = d.cross(up).normalized()
	var e2: Vector3 = d.cross(e1).normalized()
	#points for rings around a and b
	var ra: Array[Vector3]
	var rb: Array[Vector3]
	for i: int in 3: #the 3 edges of the cylinder
		var angle: float = TAU * i / 3.0 #get the angle (0, 120, and 240 degrees)
		var delta: Vector3 = (e1 * cos(angle) + e2 * sin(angle)) * r #get the point on the circle of radius r around the axis at that angle
		#add that point to both a and b sides
		ra.append(a + delta)
		rb.append(b + delta)
	for i: int in 3: #the 3 sides of the cylinder
		var n: int = (i + 1) % 3 #i represents 1 long edge of the cylinder, n is the next long edge; % 3 loops it around
		#creates triangle 1 (top-right half of the quad)
		vts.push_back(ra[i])
		vts.push_back(ra[n])
		vts.push_back(rb[n])
		#creates triangle 2 (bottom-left half of the quad)
		vts.push_back(ra[i])
		vts.push_back(rb[n])
		vts.push_back(rb[i])
