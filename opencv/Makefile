CFLAGS = `pkg-config --cflags opencv`
LIBS = `pkg-config --libs opencv`

all : select_threshold_values object_tracking qr_tracker
.PHONY : all

select_threshold_values : select_threshold_values.cpp
	g++ -fpermissive -o select_threshold_values select_threshold_values.cpp $(CFLAGS) $(LIBS)

object_tracking : object_tracking.cpp
	g++ -o object_tracking object_tracking.cpp $(CFLAGS) $(LIBS)

qr_tracker : qr_tracker.cpp
	g++ -o qr_tracker qr_tracker.cpp -lzbar $(CFLAGS) $(LIBS)

