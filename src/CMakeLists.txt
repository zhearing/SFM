include_directories(
    /usr/local/include
    /usr/include/gdal)
ADD_LIBRARY( 
	Sparse3D
        basicSfM
        matches
        essential
        triangulate
	      )

