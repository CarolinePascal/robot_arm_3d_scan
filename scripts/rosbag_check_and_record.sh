#!/bin/bash 

Help()
{
    #Display help
    echo "Script launching a safe recording of rosbag (no more 4kB rosbags)"
    echo "parameters :"
    echo "\$1 = list of recorded topics"
    echo "\$2 = rosbag output path"
}

while getopts ":h" option; do
   case $option in
      h) # display help
         Help
         exit;;
   esac
done

sleep 1
topics_to_record=$1

missing_topic=$(echo "${topics_to_record}" | tr " " "\n" | grep -v $(echo "-e $(rostopic list | tr '\n' ' ' | sed -e 's/ / -e /g' -e 's/\(.*\)-e/\1 /')"))

if [ ! -z "$missing_topic" ]; then   
  echo " "
  echo -e "\033[0;31mError: next topics are not published !\033[0m"
  echo -e "\033[0;31m${missing_topic}\033[0m"   
  echo " "   
  exit 1
fi 

#rosbag record --duration=2 -O /tmp/tmp.bag ${topics_to_record}
#
#missing_topic=$(echo "${topics_to_record}" | tr " " "\n" | grep -v $(echo "-e $(rosbag info $#{get_file_name})" | tail -n +$(rosbag info /tmp/tmp.bag) | grep -n topics | cut -f1 -d:) | sed -e #'s#.* \(\/\)#\/#g'  
#if [ ! -z "$missing_topic" ]; then   
#  echo " "   
#  echo -e "\033[0;31mError: next topics have no msg registable within a 2s span ! \033[0m"   
#  echo -e "\033[0;31m${missing_topic}\033[0m"   
#  echo " "   
#  exit 1
#fi 
#rm /tmp/tmp.bag

echo -e "\033[0;32mRecording topics $1\033[0m"

mkdir -p $(dirname $2)
rosbag record -o $2 $1
