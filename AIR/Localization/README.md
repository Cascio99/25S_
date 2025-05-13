# Step 1. Integrate GPP & Faster-LIO
    0. wGS -> UTM
    1. 초기위치: 상대 > 절대 좌표로 변환
    2. waypoints 모두 rviz 상에 표시
    3. 위 두 개에 대해서 새로운 /odom 발행 => Localization w/o offline map
    4. rosbag play 2024-...bag으로 확인 (일단 초기위치, waypoints는 실시간 취득이 아닌 임의로 설정)

## 공대 7호관 정문 gps(WGS) 값: [link](http://map.esran.com/) 참고
위도는 35.846087646108636, 경도는 127.13440823476309
- i.c. (lat, lng): (36.846087646108636, 127.13440823476309)
- waypoints(temp): [(36.846087646108636, 127.13440823476200), (36.846087646108636, 127.13440823476100), (36.846087646108636, 127.13440823476000)]

## Convert WGS into UTM: [link](https://en.wikipedia.org/wiki/Universal_Transverse_Mercator_coordinate_system#From_latitude,_longitude_(%CF%86,_%CE%BB)_to_UTM_coordinates_(E,_N))

# Reference
- [GPP_link](https://github.com/kdh044/Jbnu-Final/tree/main/global_path_planner)
- [Faster-LIO](https://github.com/gaoxiang12/faster-lio)
- [setting](https://github.com/Cascio99/25S_/tree/main/AIR/Odometry/fasterLIO)
- [robot_localization](https://docs.ros.org/en/melodic/api/robot_localization/html/integrating_gps.html)

### log
    20250512
        Done:
            1.htop CPU/RAM usage comparision(publishing vs non-publishing)
            2. WGS -> UTM "GPP + Faster-LIO localization v1"
        ToDo:
            1. UTM 정확환 좌표인지 확인할 필요 있음.
            2. Localization v2: Mark waypoints in rviz
            3. Faster-LIO modification: GPS Fusion (tuning Kalman Filter model?)
            4. Consider using `robot_localization` package
            5. waypoint 지나는 거 체크 및 위치 재조정 ?= GPS fusion