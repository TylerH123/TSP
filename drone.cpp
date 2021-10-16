//IDENTIFIER  = 1761414855B69983BD8035097EFBD312EB0527F0

#include <algorithm>
#include <iomanip>
#include <getopt.h>
#include <iostream>
#include <math.h>
#include <string>
#include <vector>
#include <limits> 

using namespace std;
const double INF = std::numeric_limits<double>::infinity();

enum Mode {
    MST,
    FASTTSP,
    OPTTSP
};

enum Place {
    Medical, 
    Border,
    Normal
};
    
class Drone {
private: 
    struct coords {
        int x; 
        int y; 
        bool visited = false; 
    };
    struct location {
        double dist = INF; 
        size_t parent; 
        Place place;
    };
    struct tspLoc {
        double dist = INF; 
        size_t parent;
    };
    vector<coords> map; 
    vector<location> prims;
    vector<size_t> tspPath;
    vector<tspLoc> primsOPT;
    vector<size_t> best;
    double distance = 0;
    double currentDist = 0;  
    double bestDistance = 0; 
    bool border = false;
    bool medical = false; 
    bool normal = false; 
    Mode mode; 


    double getTotalDistance(vector<size_t> &v) {
        double d = 0;
        for (size_t i = 0; i < map.size(); ++i) 
            d += getDistance(v[i],v[i+1]);
        return d; 
    }

    double getPrimsDistance(size_t start) {
        double sum = 0; 
        for (size_t i = start; i < primsOPT.size(); ++i) 
            sum += primsOPT[tspPath[i]].dist; 
        return sum; 
    }

    void insertPrims(int x, int y, size_t count) {
        location l; 
        if (x < 0 && y < 0) {
            l.place = Place::Medical; 
            medical = true; 
        }
        else if ((x == 0 && y <= 0) || (x <= 0 && y == 0)) {
            l.place = Place::Border; 
            border = true; 
        }
        else {
            l.place = Place::Normal; 
            normal = true; 
        }
        l.parent = count; 
        prims.push_back(l);
    }

    void readInput() {
        int vertices, x, y; 
        cin >> vertices; 
        map.reserve(vertices); 
        if (mode == Mode::MST) 
            prims.reserve(vertices); 
        else {
            if (mode == Mode::OPTTSP)
                primsOPT.resize(vertices);
            tspPath.reserve(vertices+1); 
        }
        size_t count = 0; 
        while (cin >> x >> y) {
            coords c;
            c.x = x;
            c.y = y; 
            if (mode == Mode::MST) 
                insertPrims(x,y,count++);
            map.push_back(c); 
        }
    }
    
    double getDistance(size_t idx1, size_t idx2) {
        if (idx1 == idx2) return 0;
        if (mode == Mode::MST) {
            if (prims[idx1].place == Place::Medical && prims[idx2].place == Place::Normal)
                return INF;
            if (prims[idx2].place == Place::Medical && prims[idx1].place == Place::Normal)
                return INF; 
        }
        double x = double(map[idx2].x) - double(map[idx1].x); 
        double y = double(map[idx2].y) - double(map[idx1].y); 
        return sqrt(x * x + y * y); 
    }

    double getMinDistance(size_t idx, size_t permLength) {
        double d = INF;
        for (size_t i = permLength; i < tspPath.size(); ++i) {
            double dist = getDistance(idx, tspPath[i]);
            if (dist < d) 
                d = dist;
        }
        return d; 
    }

    void primsAlgo() {
        prims[0].dist = 0;    
        size_t idx = 0;
        double dist = 0; 
        for(size_t i = 0; i < map.size(); ++i) {
            for (size_t j = 0; j < map.size(); ++j) {
                if (!map[j].visited && prims[j].dist < dist) {
                    dist = prims[j].dist;
                    idx = j; 
                }
            }
            map[idx].visited = true; 
            for (size_t j = 0; j < map.size(); ++j) {
                if (!map[j].visited) {
                    double d = getDistance(idx, j); 
                    if (d < prims[j].dist) {
                        prims[j].dist = d;
                        prims[j].parent = idx; 
                    }
                }
            }
            dist = INF; 
        }
    }

    void twoOpt() {
        for (size_t i = 0; i < map.size(); ++i) {
            for (size_t j = i+2; j < map.size(); ++j) {
                double v1v2 = getDistance(tspPath[i], tspPath[i+1]);
                double v3v4 = getDistance(tspPath[j], tspPath[j+1]); 
                double v1v3 = getDistance(tspPath[i], tspPath[j]);
                double v2v4 = getDistance(tspPath[i+1], tspPath[j+1]);
                if (v1v2 + v3v4 > v1v3 + v2v4) {
                    size_t v1 = i+1; 
                    size_t v2 = j;
                    while (v1 < v2)
                        swap(tspPath[v1++],tspPath[v2--]);
                }
            }
        }
        distance = getTotalDistance(tspPath); 
    }

    void fastTSP() {  
        size_t idx = 0,
               newIdx = 0;
        for(size_t i = 0; i < map.size() - 1; ++i) {
            map[idx].visited = true; 
            double dist = INF; 
            for (size_t j = 0; j < map.size(); ++j) {
                double d = getDistance(idx,j);
                if (!map[j].visited && d < dist) {
                    dist = d; 
                    newIdx = j; 
                }
            }
            distance += dist; 
            tspPath.push_back(idx);
            idx = newIdx; 
        }
        distance += getDistance(newIdx, 0);  
        tspPath.push_back(newIdx);
        tspPath.push_back(0);
    }

    double primsPath(size_t permLength) {
        for (size_t i = 0; i < primsOPT.size(); ++i) {
            size_t idx = tspPath[i];
            if (i < permLength)
                map[idx].visited = true; 
            else {
                map[idx].visited = false; 
                primsOPT[idx].dist = INF; 
            }
        }
        size_t idx = tspPath[permLength];
        primsOPT[idx].dist = 0;   
        double dist = 0; 
        for(size_t i = permLength; i < map.size(); ++i) {
            for (size_t j = permLength; j < map.size(); ++j) {
                size_t x = tspPath[j];
                if (!map[x].visited && primsOPT[x].dist < dist) {
                    dist = primsOPT[x].dist;
                    idx = x; 
                }
            }
            map[idx].visited = true; 
            for (size_t j = permLength; j < map.size(); ++j) {
                size_t x = tspPath[j];
                if (!map[x].visited) {
                    double d = getDistance(idx, x); 
                    if (d < primsOPT[x].dist) {
                        primsOPT[x].dist = d;
                        primsOPT[x].parent = idx; 
                    }
                }
            }
            dist = INF; 
        }
        double d = getPrimsDistance(permLength);
        return d; 
    }

    bool promising(size_t permLength) {
        if (permLength == tspPath.size()) 
            return false; 
        if (tspPath.size() - permLength < 5) 
            return true; 
        return currentDist + getMinDistance(tspPath[permLength-1], permLength) + getMinDistance(0, permLength) + primsPath(permLength) < bestDistance;
    }

    void genPerms(size_t permLength) {
        if (permLength == tspPath.size()) {
            double d = getDistance(tspPath.back(), 0);
            currentDist += d;
            if (currentDist < bestDistance) {
                best = tspPath; 
                bestDistance = currentDist; 
            }
            currentDist -= d;
        } // if
        if (!promising(permLength))
            return;
        for (size_t i = permLength; i < tspPath.size(); ++i) {
            swap(tspPath[permLength], tspPath[i]);
            currentDist += getDistance(tspPath[permLength-1], tspPath[permLength]);
            genPerms(permLength + 1);
            currentDist -= getDistance(tspPath[permLength-1], tspPath[permLength]);
            swap(tspPath[permLength], tspPath[i]);
        } // for
    } // genPerms()

    void printNode(size_t idx) {
        if (prims[idx].parent != idx) {
            if (prims[idx].parent < idx) 
                cout << prims[idx].parent << " " << idx << '\n'; 
            else 
                cout << idx << " " << prims[idx].parent << '\n'; 
        }
    }

    void printPrims() {
        double sum = 0; 
        for (auto &l : prims) 
            sum += l.dist; 
        cout << sum << '\n';
        for (size_t i = 0; i < map.size(); ++i)
            printNode(i);
    }

    void printFastTSP() {
        cout << distance << '\n';
        for (size_t i = 0; i < tspPath.size() - 1; ++i)  
            cout << tspPath[i] << " "; 
        cout << '\n';
    }

    void printOPTTSP() {
        cout << bestDistance << '\n';
        for (size_t i = 0; i < best.size(); ++i)  
            cout << best[i] << " "; 
        cout << '\n';
    }

public: 
    Drone(int argc, char *argv[]) {
        getMode(argc, argv);
        readInput(); 
        if (mode == Mode::MST) {
            if (medical && !border && normal) {
                cerr << "Cannot construct MST" << '\n';
                exit(1);
            }
            primsAlgo(); 
            printPrims(); 
        }
        if (mode == Mode::FASTTSP) {
            fastTSP(); 
            twoOpt();
            printFastTSP(); 
        }
        if (mode == Mode::OPTTSP) {
            fastTSP(); 
            twoOpt();
            tspPath.pop_back();
            best = tspPath;
            bestDistance = distance; 
            genPerms(1);
            printOPTTSP();
        }
    }

	// Print help for the user when requested.
    // argv[0] is the name of the currently executing program
    void printHelp(char *argv[]) {
        cout << "This program is simulate a star wars battle" << '\n';
        cout << "Usage: " << argv[0] << " -m" << '\n';
    } // printHelp()

    void getMode(int argc, char * argv[]) {
        // These are used with getopt_long()
        opterr = false; // Let us handle all error output for command line options
        int choice;
        int option_index = 0;
        string in_mode;
        option long_options[] = {
            // Fill in two lines, for the "mode" ('m') and
            // the "help" ('h') options.
            { "mode", required_argument,      nullptr, 'm'},
            { "help",  no_argument,        nullptr, 'h'},
            { nullptr, 0,                  nullptr, '\0'}
        };
        // cout << getopt_long(argc, argv, "m:h", long_options, &option_index) << endl; 
        // Fill in the double quotes, to match the mode and help options.
        while ((choice = getopt_long(argc, argv, "m:h", long_options, &option_index)) != -1) {
            switch (choice) {
            case 'h':
                printHelp(argv);
                exit(0);
            case 'm':
                in_mode = optarg;
                if (in_mode == "") {
                    cerr << "No mode specified" << '\n';
                    exit(1); 
                }
                if (in_mode == "MST") 
                    mode = Mode::MST; 
                else if (in_mode == "FASTTSP")
                    mode = Mode::FASTTSP; 
                else if (in_mode == "OPTTSP") 
                    mode = Mode::OPTTSP;
                else  {
                    cerr << "Invalid mode" << '\n';
                    exit(1); 
                }
                break;
            default:
                cerr << "Invalid command line option" << endl;
                exit(1);
            } // switch
        } // while
    } // getMode()
};

int main(int argc, char *argv[]) {
    // This should be in all of your projects, speeds up I/O
    ios_base::sync_with_stdio(false);
    cout << setprecision(2); 
    cout << fixed;
    Drone d(argc, argv);
    return 0;
} // main()
