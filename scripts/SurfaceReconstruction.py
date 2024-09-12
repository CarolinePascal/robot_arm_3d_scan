import open3d as o3d
from meshlib import mrmeshpy
import meshlib.mrmeshnumpy as mrmeshnumpy
import pymeshfix

import trimesh
import meshio

import coacd
import numpy as np

import cloup

@cloup.command()
@cloup.option("--input_point_cloud", type=str, default="PointCloudICP.pcd", help="Input point cloud")
@cloup.option("--offset_value", type=float, default=0.05, help="Mesh offset value")
@cloup.option("--resolution", type=float, default=0.05, help="Mesh resolution")
@cloup.option("--show", is_flag=True, help="Shows intermediate meshes")
def main(input_point_cloud, offset_value, resolution, show):

    point_cloud = o3d.io.read_point_cloud(input_point_cloud)

    ### SURFACE RECONSTRUCTION ###

    #mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(point_cloud, depth=10)
    
    init_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(point_cloud, 0.02)  
    vclean, fclean = pymeshfix.clean_from_arrays(init_mesh.vertices, init_mesh.triangles)
    init_mesh = o3d.geometry.TriangleMesh(vertices=o3d.utility.Vector3dVector(vclean), triangles=o3d.utility.Vector3iVector(fclean))
    
    # radii = [0.005, 0.01, 0.02, 0.04, 0.08]
    # mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(point_cloud, o3d.utility.DoubleVector(radii))
    
    if(show):
        init_mesh.compute_vertex_normals()
        o3d.visualization.draw_geometries([init_mesh])

    o3d.io.write_triangle_mesh("MeshICP.stl", init_mesh)

    ### COLLISION MESH ###

    collision_offset_value = 0.01
    collision_resolution = 0.01

    collision_mesh = mrmeshpy.loadMesh("MeshICP.stl")

    collision_offset_params = mrmeshpy.OffsetParameters()
    collision_offset_params.voxelSize = collision_resolution

    # (0) Unsigned : unsigned distance, useful for bidirectional Shell offset
    # (1) OpenVDB : sign detection from OpenVDB library, which is good and fast if input geometry is closed
    # (2) ProjectionNormal : the sign is determined based on pseudonormal in closest mesh point (unsafe in case of self-intersections)
    # (3) WindingRule : ray intersection counter, significantly slower than ProjectionNormal and does not support holes in mesh
    # (4) HoleWindingRule : computes winding number generalization with support of holes in mesh, slower than WindingRule 
    collision_offset_params.signDetectionMode = mrmeshpy.SignDetectionMode.OpenVDB

    collision_offset_mesh = mrmeshpy.offsetMesh(collision_mesh, collision_offset_value, collision_offset_params)

    mrmeshpy.saveMesh(collision_offset_mesh,"StudiedObject.stl")

    tmp_mesh = trimesh.load_mesh("StudiedObject.stl")  
    tmp_mesh = coacd.Mesh(tmp_mesh.vertices, tmp_mesh.faces)
    result = coacd.run_coacd(tmp_mesh,
                             threshold=0.15,
                             max_convex_hull=20,
                             preprocess_mode="auto",
                             preprocess_resolution=50,
                             resolution=2000,
                             mcts_nodes=20,
                             mcts_iterations=150,
                             mcts_max_depth=3,
                             pca=False,
                             merge=True)

    mesh_parts = []
    for vs, fs in result:
        mesh_parts.append(trimesh.Trimesh(vs, fs))

    scene = trimesh.Scene()
    for p in mesh_parts:
        scene.add_geometry(p)
    scene.export("StudiedObject.stl")

    ### CROPPING ###

    mesh_centroid = np.mean(np.asarray(init_mesh.vertices), axis=0)
    mesh_size = np.linalg.norm(init_mesh.get_max_bound()-init_mesh.get_min_bound())/(2*np.sqrt(3))

    scene = trimesh.Scene()
    for vs, fs in result:
        local_mesh = trimesh.Trimesh(vs, fs)
        _,d,_ = trimesh.proximity.closest_point(local_mesh, [mesh_centroid])
        if(d < 0.1*mesh_size):
            scene.add_geometry(local_mesh)

    crop_mesh = init_mesh.crop(o3d.geometry.AxisAlignedBoundingBox(min_bound=[scene.centroid[0]-scene.extents[0]/2, scene.centroid[1]-scene.extents[1]/2, scene.centroid[2]-scene.extents[2]/2], max_bound=[scene.centroid[0]+scene.extents[0]/2, scene.centroid[1]+scene.extents[1]/2, scene.centroid[2]+scene.extents[2]/2]))

    if(show):
        o3d.visualization.draw_geometries([crop_mesh])

    o3d.io.write_triangle_mesh("MeshICP_Cropped.stl", crop_mesh)
            
    ### OFFSETTING, SMOOTHING & DECIMATION ###

    #Step 1 : Offset surface

    mesh = mrmeshpy.loadMesh("MeshICP_Cropped.stl")

    offset_params = mrmeshpy.OffsetParameters()
    offset_params.voxelSize = resolution/10  #More than expected resolution (decimate afterwards)

    # (0) Unsigned : unsigned distance, useful for bidirectional Shell offset
    # (1) OpenVDB : sign detection from OpenVDB library, which is good and fast if input geometry is closed
    # (2) ProjectionNormal : the sign is determined based on pseudonormal in closest mesh point (unsafe in case of self-intersections)
    # (3) WindingRule : ray intersection counter, significantly slower than ProjectionNormal and does not support holes in mesh
    # (4) HoleWindingRule : computes winding number generalization with support of holes in mesh, slower than WindingRule 
    offset_params.signDetectionMode = mrmeshpy.SignDetectionMode.OpenVDB

    offset_mesh = mrmeshpy.offsetMesh(mesh, offset_value, offset_params)

    if(show):
        verts = mrmeshnumpy.getNumpyVerts(mesh)
        faces = mrmeshnumpy.getNumpyFaces(mesh.topology)

        display = trimesh.Trimesh(verts, faces)
        display.visual.vertex_colors = trimesh.visual.random_color()

        offset_verts = mrmeshnumpy.getNumpyVerts(offset_mesh)
        offset_faces = mrmeshnumpy.getNumpyFaces(offset_mesh.topology)

        scene = trimesh.scene.Scene()
        scene.add_geometry(display)
        offset_display = trimesh.Trimesh(offset_verts, offset_faces)
        offset_display.visual.vertex_colors = trimesh.visual.random_color()
        offset_display.visual.vertex_colors[:,-1] = 126
        scene.add_geometry(offset_display)
        scene.show()

    #Step 2 : Smooth surface

    smooth_params = mrmeshpy.MeshRelaxParams()
    smooth_params.force = 0.2   #Between 0.0 and 0.5
    smooth_params.iterations = 5

    smooth_result = mrmeshpy.relaxKeepVolume(offset_mesh,smooth_params)

    if(show):
        smooth_verts = mrmeshnumpy.getNumpyVerts(offset_mesh)
        smooth_faces = mrmeshnumpy.getNumpyFaces(offset_mesh.topology)

        scene = trimesh.scene.Scene()
        scene.add_geometry(display)
        smooth_display = trimesh.Trimesh(smooth_verts, smooth_faces)
        smooth_display.visual.vertex_colors = trimesh.visual.random_color()
        smooth_display.visual.vertex_colors[:,-1] = 126
        scene.add_geometry(smooth_display)
        scene.show()

    #Step 3 : Decimate surface (adapt mesh size)
    
    # Setup decimate parameters
    decimate_settings = mrmeshpy.DecimateSettings()
    decimate_settings.strategy = mrmeshpy.DecimateStrategy.ShortestEdgeFirst
    decimate_settings.maxError = 10e10
    decimate_settings.maxEdgeLen = resolution
    decimate_settings.tinyEdgeLength = resolution
    
    # Decimate mesh
    decimate_result = mrmeshpy.decimateMesh(offset_mesh, decimate_settings)

    if(show):
        decimate_verts = mrmeshnumpy.getNumpyVerts(offset_mesh)
        decimate_faces = mrmeshnumpy.getNumpyFaces(offset_mesh.topology)

        scene = trimesh.scene.Scene()
        scene.add_geometry(display)
        decimate_display = trimesh.Trimesh(decimate_verts, decimate_faces)
        decimate_display.visual.vertex_colors = trimesh.visual.random_color()
        decimate_display.visual.vertex_colors[:,-1] = 126
        scene.add_geometry(decimate_display)
        scene.show(flags={'wireframe': True})

    mrmeshpy.saveMesh(offset_mesh,"OffsetMesh_" + str(offset_value) + "_" + str(resolution) + ".stl")
    final_mesh = trimesh.load_mesh("OffsetMesh_" + str(offset_value) + "_" + str(resolution) + ".stl")  

    if(show):
        circle = trimesh.primitives.Sphere(0.25,center=np.mean(final_mesh.vertices,axis=0))
        circle.visual.vertex_colors = [0,0,0,63]

        init_mesh_plot = trimesh.Trimesh(vertices=init_mesh.vertices, faces=init_mesh.triangles)
        init_mesh_plot.visual.vertex_colors = trimesh.visual.random_color()
        final_mesh.visual.vertex_colors = trimesh.visual.random_color()
        final_mesh.visual.vertex_colors[:,-1] = 126

        scene = trimesh.Scene()

        scene.add_geometry(init_mesh_plot)
        scene.add_geometry(final_mesh)
        scene.add_geometry(circle)
        scene.show() 

    print("Final vertices number : ", len(final_mesh.vertices))
    print("Final resolution : ", final_mesh.edges_unique_length.max())

    # Save final mesh
    meshio.write_points_cells("MeasurementsMesh.mesh", final_mesh.vertices, [("triangle",final_mesh.faces)])
    final_mesh.export("StudiedObject.stl")

    
if __name__ == "__main__":
    main()









