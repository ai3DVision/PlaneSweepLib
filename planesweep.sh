#!/bin/sh  

if [ -d "testResults/pinholeCamera/" ]; then
  echo "Old results found. They need to be deleted before running the tests again, do you want to delete them?"
  rm -r -I testResults/pinholeCamera
fi

if [ ! -d "testResults/pinholeCamera/" ]; then
  mkdir -p testResults/pinholeCamera/

  echo "Runing pinhole planesweep on the niederdorf1 dataset..."
  build/bin/planesweep --dataFolder /is/ps2/dpaschalidou/Datasets/probabilistic_reconstruction_data/BH` --reference_idx 5 --n_views 4
  mv pinholeTestResults testResults/pinholeCamera/niederdorf1
  echo "done" 
fi
