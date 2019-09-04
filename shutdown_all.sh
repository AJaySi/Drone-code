# TBD : Need a better cleanup. Processes are left dangling.
echo "Killing all that it spawned."
kill -9 $(ps -eF | grep mv | awk -F' ' '{print $2}')
kill -9 $(ps -eF | grep QGC | awk -F' ' '{print $2}')
kill -9 $(ps -eF | grep ardu | awk -F' ' '{print $2}')
