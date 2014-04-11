#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <cstdlib>
#include <cstring>

using namespace std;

struct TagGCP {
	int id;
	int x;
	int y;
};

int main(int argc, const char* argv[]) {
	ifstream in;
	ostringstream oss;
	char filename[30];
	string img;
	TagGCP tag;
	int count = 0;
	bool geowarp = false;
	if (strcmp(argv[1], "true")) 
		geowarp = true;

	for (int i = 2; i < argc; ++i) {
		img = argv[i];
		img = img.substr(0, img.length()-4);
		cout << img << endl;
		sprintf(filename, "%s", argv[i]);
		in.open(filename);
		oss << "./gdal_translate -of gtiff -co \"compress=LZW\" ";
		while (in >> tag.id >> tag.x >> tag.y) {
			//if (count >= 5) break;
			++count;
			cout << "Tag ID: " << tag.id << ", " << tag.x 
				<< ", " << tag.y << endl;
			oss << " -gcp " << tag.x << " " << tag.y << " " 
				<< tag.id % 6 << " " << tag.id / 6;
		}
		oss << " -a_srs \"wgs84\" " << img << ".tif " 
			<< img << "geo.tif";
		cout << oss.str() << endl;
		if (count >= 3) {
			system(oss.str().c_str());
			oss.flush();
			oss.str("");
			oss.clear();

			if (geowarp) {
				oss << "./gdalwarp -dstnodata \"255 0 0\" -overwrite -order 1 " << img << "geo.tif " << img << "newgeo.tif ";
				system(oss.str().c_str());
			}
		}
		oss.flush();
		oss.str("");
		oss.clear();
		in.close();
		count = 0;
		//system("./gdal_translate -of gtiff -co \"compress=LZW\" -gcp 
	}
	if (geowarp) {
		oss.flush();
		oss.str("");
		oss.clear();
		oss << "./gdalwarp -dstnodata 0 -srcnodata \"255 0 0\" -overwrite -order 1 *newgeo.tif out.tif";
		system(oss.str().c_str());
	}
}
