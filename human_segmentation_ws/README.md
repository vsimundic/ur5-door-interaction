# Establishment of the human segmentation

1. In the directory `human_segmentation_ws/src/human_seg/config` there is a file `config.yaml` which contains all configuration parameters. Set `load_path_root` and `save_path_root` to your choice. `load_path_root` is a directory where your bagfiles are stored. `save_path_root` is a directory where your segmented images will be saved. The other parameters do not need to be changed.

2. In the `load_path_root` create a file called `bags_sequences.txt` where you should write the names of your bagfiles. You can stop the sequence of loading bagfiles by putting the keyword `end` after the bagfile names or by simply ending the file. An example of the `bags_sequences.txt` file might look like this:
```txt 
01_night_stand_human_2022-11-21-15-40-51.bag 
02_door_close_03_2022-11-23-10-16-41.bag 
end
```

3. Open a terminal (somewhere in the container) and type:
```bash 
roscore
```

4. Open a second terminal in the `human_segmentation_ws` directory and type:
```bash 
source devel/setup.bash 
rosrun human_seg segment_videos.py
```