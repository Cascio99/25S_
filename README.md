## traj
 - 1.png: default vs capacity_10000
 - 2.png: default vs capacity_1000000

## MEM_usage ( ave / max )
- default vs non_publish_mode(point) vs capacity_100000
    ### default( MB )
        ROS Launch    	    43.74	43.76
        FASTER-LIO Core	    423.49	699.12
        ROS Master	        40.77	40.79
        ROS Out	            10.62	10.99
        Laser Mapping	    379.75	655.49
        RViz Visualization	964	    1744.41
        ROS Bag Player	    72.09	72.82
    ### non_publish_mode
        ROS Launch	        44.12	44.12
        FASTER-LIO Core	    154.11	189.62
        ROS Master	        40.8	40.8
        ROS Out	            10.95	10.95
        Laser Mapping	    109.99	145.5
        RViz Visualization	149.75	152.67
        ROS Bag Player		72.5	72.5
    ### capacity_100000
        ROS Launch	        43.6	43.6
        FASTER-LIO Core     146.82	165.57
        ROS Master	        41.08	41.08
        ROS Out	            10.86	10.86
        Laser Mapping	    103.23	121.98
        RViz Visualization	148.77	152.47
        ROS Bag Player		72.59	72.59
        
## runtime
 default vs non_publish_mode(point) vs capacity_100000
 - IEKF Solve and Update: 8.2 ms vs 7.0 ms vs 6.5 ms