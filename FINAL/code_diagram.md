# Key configs


#### ***** options_.capacity: ivox3d::line53~57
    max # of voxels(grids) the map can hold!
    When the # of voxels exceseds capacity, "THE OLDEST ARE PRUNED" from voxel hash map(grids_map_)
    AddPoints() in MapIncremental(): ivox3d::line271~274
    value: 1000000 -> ?
    smaller value = lower memory usage
#### *** cube_side_length: 1000
    side length of the localmap
    smaller value = lower memory usage
#### point_filter_num: 3
    # of times to apply the point cloud filter (i.e. outlier removal / downsampling)
#### max_iteration: 3
    max # of iterations for the main optimization loop: kf_.update..()
#### filter_size_surf: 0.5
    voxel size for downsampling surface feature points in the scan
    Larger value = aggressive downsampling = fewer points = lower memory usage
    voxel_scan_.setLeatSize()
#### filter_size_map: 0.5
    voxel size for downsampling points when add to map
    If 0, all downsampled scan points are added to the localmap
    MapIncremental()
    Larger value = lower memory usage
#### ivox_grid_resolution: 0.5
    size of each voxel(grid) in the ivox_(localmap).
    ivox3d::Pos2Grid() in GetClosestPoint()
    smaller resolution = lower memory usage = coarser map
#### ivox_nearby_type: 18             // 6, 18, 26
#### esti_plane_threshold: 0.1
    esti_plane() in ObsModel(): threshold for considering a plane wrt nn point of current scan
    smaller threshold = less points selected = lower memory usage

# System Flow

- InitRos()
    - ivox_: localmap init
    - kf_.init_dyn_share(): Iterated-Error-State-Extended-Kalman-Filter(IESEKF) init
        - ObsModel() init
- Run()
    - p_imu_->Process(): IMU process, kf prediction, undistortion
        - IMUInit():
            - change_x()
            - change_P()
        - UndistortPcl(): kf prediction, undistortion
    - kf_.update_iterated_dyn_share_modified(): iterate up to 3(max_iteration) until dyn_share_.converge()
        - ObsModel(): update IESEKF
            - getVector3fMap(): transform to world frame
            - GetClosestPoint(): find the closest surfaces in the localmap
            - esti_plane(): estimate a plane
            - ekfom_data.h(i) = -corr_pts_[i][3]
              - H: Measurement Jacobian Matrix
              - Measurement: distance to the closest surface/corner
    - MapIncremental(): Incremental local mapping
      - PointBodyToWorld(): transform from body to world frame
      - emplace_back(): add points(points_to_add & point_no_need_downsample) for local map
      - ivox_->AddPoints(): add points to localmap
    - publish topics


# [code diagram](https://www.mermaidchart.com/)

```mermaid
---
config:
  theme: default
---
flowchart TD
  Root["laser_mapping.cc"] --> A["InitROS"] & B["Run"]
    A --> AA["localmap init"] & AB["kf_.init_dyn_share"]
      AA --- AAA(["ivox_"])
        AAA --- AAAA(["filter_size_map"]) & AAAB(["ivox_grid_resolution"]) & AAAC(["ivox_nearby_type"])
        AAA -.- etc(["etc"])
          AAAA ---> ABAA["point_world.getVector3fMap"]
            ABAA --- ABAA_[Transform points from Body to World Frame]
      AB ------> ABA["ObsModel"]
        ABA ---> ABAA
        ABA ---> ABAB["GetClosestPoint"]
          ABAB --- ABAB_[Find the nearest points in the localmap]
        ABA ---> ABAC["common::esti_plane"]
          ABAC --- ABAC_[For each set of nearest points, estimates a plane]
        ABA ---> ABAD["residuals_[i] <- pd2"]
          ABAD --- ABAD_[The residual, 'distance to the plane' is computed]
        ABA ---> ABAE["ekfoom_data.h_x.block<1, 12>(i, 0) << norm_vec[0].."]
          ABAE --- ABAE_[Jacobian is built based on the residuals and the normal vector of the plane]
    B --> BA["SyncPackages"]
    B ---> BB["p_imu_.Process"] & BC["Downsampling"]
      BC --- BCA(["scan_undistort"]) & BCB(["scan_down_body"])
  Root ---------> C["IESEKF update"] & D["MapIncremental"]
    C --> CA["kf_.update_dyn_share_modified"]
    CA ---> ABA
    D --> DA["PointBodyToWorld"]
      DA --- DAA(["points_to_add"]) & DAB(["points_no_need_downsample"])
        DAB <----- AAAA
    D --> DB["emplace_back(point_world)"]
      DAA -.-> DB
      DAB -.-> DB
    D --------> DC["AddPoints"]
      AAA -.-> DC