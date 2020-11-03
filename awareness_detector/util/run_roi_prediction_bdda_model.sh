#!/bin/bash

predictor_dir="$HOME"/test/roi-prediction
source $predictor_dir/bdda/venv/bin/activate
PYTHONPATH=$PYTHONPATH:$predictor_dir

FILES="$HOME"/catkin_ws_carla/src/gaze-detector/awareness_detector/output_h5_to_rosbag/*
h5_extracted_data="$HOME"/h5_out
output_dir="$HOME"/out/roi-bdda
qualitys=(100 50 20 10)

for f in $FILES; do
    echo $h5_extracted_data/$(basename ${f%.h5.bag})
    python3 $predictor_dir/bdda/prepare.py $h5_extracted_data/$(basename ${f%.h5.bag}) --image_topics 'front' -s .jpg
    mkdir -p ${output_dir}/$(basename ${f%.h5.bag})/gaze_maps_gt
    cp $predictor_dir/bdda/data/gazemap_images/* ${output_dir}/$(basename ${f%.h5.bag})/gaze_maps_gt

    for q in "${qualitys[@]}"; do
        echo "Processing $(basename ${f%.h5.bag}) file..."

        mkdir -p ${output_dir}/$(basename ${f%.h5.bag})/$q
        roslaunch awareness_detector extract.launch input_rosbag:=$f output_folder:=${output_dir}/$(basename ${f%.h5.bag})/$q img_quality:=$q

        python3 $predictor_dir/bdda/prepare.py $output_dir/$(basename ${f%.h5.bag})/$q --image_topics 'front' -s .jpg
        cp $predictor_dir/bdda/data/camera_images/* $predictor_dirn/bdda/data/testing/camera_images
        python $predictor_dir/bdda/generate_fake_gazemaps.py $predictor_dir/bdda/data/testing/camera_images -o $predictor_dir/bdda/data/testing/gazemap_images/

        $predictor_dir/bdda/test.sh
        python $predictor_dir/bdda/reformat_gaze_maps.py $predictor_dirn/bdda/driver_attention_prediction/pretrained_models/model_for_inference/prediction_iter_0/ $predictor_dir/bdda/data/naming.json -s .jpg -o ${output_dir}/$(basename ${f%.h5.bag})/$q
        python $predictor_dir/annotation/generate_pseudo_label.py -s .jpg ${output_dir}/$(basename ${f%.h5.bag})/$q/$q -o ${output_dir}/$(basename ${f%.h5.bag})/$q

        python3 $predictor_dir/bdda/prepare.py $output_dir/$(basename ${f%.h5.bag})/$q --image_topics 'front' -s .jpg
        mkdir ${output_dir}/$(basename ${f%.h5.bag})/$q/gaze_maps
        cp $predictor_dir/bdda/data/gazemap_images/* ${output_dir}/$(basename ${f%.h5.bag})/$q/gaze_maps
        echo "Calculating error"
        python calculate_error.py ${output_dir}/$(basename ${f%.h5.bag})/$q/gaze_maps ${output_dir}/$(basename ${f%.h5.bag})/gaze_maps_gt ${output_dir}/$(basename ${f%.h5.bag}) $q

        rm -f $predictor_dir/bdda/data/naming.json
        rm -f $predictor_dir/bdda/data/camera_images/*
        rm -f $predictor_dir/bdda/data/gazemap_images/*
        rm -f $predictor_dir/bdda/data/testing/camera_images/*
        rm -f $predictor_dir/bdda/data/testing/gazemap_images/*
        rm -f $predictor_dir/bdda/driver_attention_prediction/pretrained_models/model_for_inference/prediction_iter_0/*
        #rm -f prediction_data/*
    done
done
