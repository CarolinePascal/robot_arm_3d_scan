import open3d as o3d
from meshlib import mrmeshpy
import meshlib.mrmeshnumpy as mrmeshnumpy
import pymeshfix

import trimesh
import meshio

import coacd
import numpy as np

if __name__ == "__main__":
    point_cloud = o3d.io.read_point_cloud("PointCloudICP.pcd")

    ### SURFACE RECONSTRUCTION ###

    #mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(point_cloud, depth=10)
    
    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(point_cloud, 0.02)
    mesh.compute_vertex_normals()
    vclean, fclean = pymeshfix.clean_from_arrays(mesh.vertices, mesh.triangles)
    mesh = o3d.geometry.TriangleMesh(vertices=o3d.utility.Vector3dVector(vclean), triangles=o3d.utility.Vector3iVector(fclean))
    
    # radii = [0.005, 0.01, 0.02, 0.04, 0.08]
    # mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(point_cloud, o3d.utility.DoubleVector(radii))
    
    mesh.compute_vertex_normals()
    #o3d.visualization.draw_geometries([mesh])
    o3d.io.write_triangle_mesh("MeshICP.stl", mesh)

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
                             max_convex_hull=5,
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

    mesh_centroid = np.mean(np.asarray(mesh.vertices), axis=0)
    mesh_size = np.linalg.norm(mesh.get_max_bound()-mesh.get_min_bound())/(2*np.sqrt(3))

    crop_mesh = mesh.crop(o3d.geometry.AxisAlignedBoundingBox(min_bound=[mesh_centroid[0]-mesh_size, mesh_centroid[1]-mesh_size, mesh_centroid[2]-mesh_size], max_bound=[mesh_centroid[0]+mesh_size, mesh_centroid[1]+mesh_size, mesh_centroid[2]+mesh_size]))

    # o3d.visualization.draw_geometries([crop_box])
    o3d.io.write_triangle_mesh("MeshICP_Cropped.stl", crop_mesh)
            
    ### OFFSETTING, SMOOTHING & DECIMATION ###

    offset_value = 0.05
    resolution = 0.05

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

    # verts = mrmeshnumpy.getNumpyVerts(mesh)
    # faces = mrmeshnumpy.getNumpyFaces(mesh.topology)

    # display = trimesh.Trimesh(verts, faces)
    # display.visual.vertex_colors = trimesh.visual.random_color()

    # offset_verts = mrmeshnumpy.getNumpyVerts(offset_mesh)
    # offset_faces = mrmeshnumpy.getNumpyFaces(offset_mesh.topology)

    # scene = trimesh.scene.Scene()
    # scene.add_geometry(display)
    # offset_display = trimesh.Trimesh(offset_verts, offset_faces)
    # offset_display.visual.vertex_colors = trimesh.visual.random_color()
    # offset_display.visual.vertex_colors[:,-1] = 126
    # scene.add_geometry(offset_display)
    # scene.show()

    #Step 2 : Smooth surface

    smooth_params = mrmeshpy.MeshRelaxParams()
    smooth_params.force = 0.2   #Between 0.0 and 0.5
    smooth_params.iterations = 5

    smooth_result = mrmeshpy.relaxKeepVolume(offset_mesh,smooth_params)

    # smooth_verts = mrmeshnumpy.getNumpyVerts(offset_mesh)
    # smooth_faces = mrmeshnumpy.getNumpyFaces(offset_mesh.topology)

    # scene = trimesh.scene.Scene()
    # scene.add_geometry(display)
    # smooth_display = trimesh.Trimesh(smooth_verts, smooth_faces)
    # smooth_display.visual.vertex_colors = trimesh.visual.random_color()
    # smooth_display.visual.vertex_colors[:,-1] = 126
    # scene.add_geometry(smooth_display)
    # scene.show()

    #Step 3 : Decimate surface (adapt mesh size)
    
    # Setup decimate parameters
    decimate_settings = mrmeshpy.DecimateSettings()
    decimate_settings.strategy = mrmeshpy.DecimateStrategy.ShortestEdgeFirst
    decimate_settings.maxError = 10e10
    decimate_settings.maxEdgeLen = resolution
    decimate_settings.tinyEdgeLength = resolution
    
    # Decimate mesh
    decimate_result = mrmeshpy.decimateMesh(offset_mesh, decimate_settings)

    # decimate_verts = mrmeshnumpy.getNumpyVerts(offset_mesh)
    # decimate_faces = mrmeshnumpy.getNumpyFaces(offset_mesh.topology)

    # scene = trimesh.scene.Scene()
    # scene.add_geometry(display)
    # decimate_display = trimesh.Trimesh(decimate_verts, decimate_faces)
    # decimate_display.visual.vertex_colors = trimesh.visual.random_color()
    # decimate_display.visual.vertex_colors[:,-1] = 126
    # scene.add_geometry(decimate_display)
    # scene.show(flags={'wireframe': True})

    mrmeshpy.saveMesh(offset_mesh,"OffsetMesh.stl")

    final_mesh = trimesh.load_mesh("OffsetMesh.stl")  
    final_mesh.show(flags={'wireframe': True})

    print("Measurements points (P1) : ", len(final_mesh.vertices))
    print("Final resolution : ", final_mesh.edges_unique_length.max())

    # Save final mesh
    meshio.write_points_cells("MeasurementsMesh.mesh", final_mesh.vertices, [("triangle",final_mesh.faces)])
    #final_mesh.export("StudiedObject.stl")

    

    










