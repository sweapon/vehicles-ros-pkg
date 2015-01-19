#!/bin/bash
URL=https://bitbucket.org/labust/labust-data/downloads/pladypos_meshes.tar.bz2
username=labust-unizg
SCRIPT_DIR="$( cd "$(dirname "$0")" ; pwd -P )"
echo ${SCRIPT_DIR}
cd ${SCRIPT_DIR}

if [ $# -eq 1 ]
 then
   username=$1
fi

downloaded=0

if [ -f `basename ${URL}` ]
 then
   echo "File already downloaded."
   downloaded=1
 else
   if wget --user $username --ask-password ${URL};
     then
       echo "Dowloaded"
       downloaded=1
    else
      echo "Download failed"
   fi
fi

if [ $downloaded == 1 ]
 then
  echo "Unpacking ..."
  (exec tar xvjf `basename ${URL}` -C ${SCRIPT_DIR}/..)
  echo "Do you wish to remove the archive `basename ${URL}`"
  select yn in "Yes" "No"; do
    case $yn in
	Yes ) rm `basename ${URL}`; break;;
	No ) break;;
    esac
  done
fi

