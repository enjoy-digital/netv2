#!/bin/sh
# TODO: use udev instead

FOUND=$(lsmod | grep netv2)
if [ "$FOUND" != "" ] ; then
    echo "Module already installed"
    exit 0
fi

INS=$(insmod netv2.ko 2>&1)
if [ "$?" != "0" ] ; then
    ERR=$(echo $INS | sed -s "s/.*netv2.ko: //")
    case $ERR in
    'Invalid module format')
        set -e
        echo "Kernel may have changed, try to rebuild module"
        make -s clean
        make -s
        insmod netv2.ko
        set +e
        ;;
    'No such file or directory')
        set -e
        echo "Module not compiled"
        make -s
        insmod netv2.ko
        set +e
        ;;
    'Required key not available')
        echo "Can't insert kernel module, secure boot is probably enabled"
        echo "Please disable it from BIOS"
        exit 1
        ;;
    *)
        >&2 echo $INS
        exit 1
    esac
fi

major=$(awk '/ netv2$/{print $1}' /proc/devices)
for i in `seq 0 4` ; do
    rm -f "/dev/netv2$i"
    mknod -m 666 /dev/netv2$i c $major $i
done

