if [ -e mybuild ]
then
    cd mybuild
    rm -r ./*
else
    mkdir mybuild
    cd mybuild
fi
cmake ..
echo "CMAKE FINISH !"
make
echo "MAKE FINISH !"



