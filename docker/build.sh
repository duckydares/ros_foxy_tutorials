#!/usr/bin/env bash

if [ $# == 0 ]
then
    echo "Usage $0 directory-name"
    exit 1
fi

# get path to current directory
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

if [ ! -d $DIR/$1 ]
then
  echo "image-name must be a directory in the same folder as this script"
  exit 2
fi

# Get user id and make image name / tag
user_id=$(id -u)
image_name=$(basename $1)
image_name_plus_tag=$image_name:latest-$(date +%F_%H%M)

# Grab the nvidia driver version
nvidia_version=$(modinfo nvidia | grep version:)
nvidia_version=${nvidia_version//srcversion:*/}
nvidia_version=${nvidia_version//version: /}
nvidia_version=${nvidia_version//.*/}

shift

docker build \
  --rm \
  --build-arg user_id=$user_id \
  --build-arg nvidia_version \
  -t $image_name_plus_tag \
  -f $DIR/$image_name/Dockerfile .
docker tag $image_name_plus_tag $image_name:latest

echo "Built $image_plus_tag and tagged as $image_name:latest"