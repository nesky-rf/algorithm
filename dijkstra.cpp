// Implementation of Dijkstra's algorithm
// dijkstra.cpp - Jordi Capdevila
// Dec. 7, 2020
//

#include <iostream>
#include <iomanip>	// setw()
#include <vector>	// vector
#include <cstdlib>	// rand()
#include <cassert>	// assert()

using namespace std;

// Generates and randomly initializes a matrix of (n,n) nodes. Edge values 
// span from [0.0,10.0), as doubles.
// Example:
// 	Graph g(10);
class Graph {
public:
    explicit Graph(int n=5);	// n takes number of nodes and allocates space for nxn matrix
    ~Graph() {delete matrix;}	// frees allocated space by matrix
    void print();		// outputs whole matrix with rows as node paths and columns as intersections to other nodes
    int V() {return nodes;}	// return rows of matrix
    int E();			// return number of non-zero edges, to perform density matrix
    bool adjacent(int i, int j) {return (*matrix)[i][j]>0;}	// checks if two nodes are neighbors
    vector<int> neighbors(int node);	// returns vector of indexes of available paths / neighbors of node 
    vector<double> get_node_value(int node);	// returns vector of edges of available paths / neighbors of node 
    void set_node_value(int node, vector<double> val);	// sets node edge values
    double get_edge_value(int node, int path);		// gets node edge values, neighbors or not
    void set_edge_value(int node, int path, double val);// sets node and associated path / intersection to a value

private:
    vector<vector<double>>* matrix;	// generetated undirected graph matrix; symmetrical [i][j]=[j][i], where i,j are
					// nodes; and hollow [i][i] = 0, i.e. no path to node itself
    int nodes;
};

// Helps to keep track of visited and unvisted nodes. Allocates space for n
// node elements in the queue.
// Example:
//	Queue q(G.V());
class Queue {
public:
    explicit Queue(int n=5);	// takes as input number of nodes to generate visit and unvisit queue
    ~Queue() {delete unvisit,visit;}	// frees allocated space by queue
    void print();		// print both queues: visit and unvisit
    void remove(int element);	// removes elmement from unvisited queue
    bool top() {return (*unvisit).front();}	// top element of unvist queue
    int size() {return (*unvisit).size();}	// size of unvisit queue
    int contains(int element);			// returns -1 if element not in unvisit queue or index location
    bool empty() {return (*unvisit).empty();}	// returns 1 when no elements left in unvisit queue
private:
    vector<int>* unvisit;	// queue initalized to nodes value
    vector<int>* visit;		// stores removed elements from unvisit queue
    int nodes;
};

// Performs Djikstra's algorithm on a speciifed node (start) to a graph represented by n x n matrix. 
// Outputs the resulting path matrix with column 0 with node number, column 1 length to start and
// column 2 previous path.
// Example:
//	Djikstra d(Graph g, Queue q, start_node);
// Output example:
// node    len  prev
//     0     0     0
//     1   7.1     0
//     2  13.7     1
//     3   4.3     0
//     4  21.1     2
// i.e. start_node is 0 (total=0), path from node(0) -> node(4): 0 -> 1 -> 2 -> 4, 
// and path length from(0) to(4) = 7.1 + 13.7 + 21.1
// Note: when len = NA, means unreachable node
class Djikstra {
public:
    Djikstra(Graph &G, Queue &Q, int m);	// initializes algorithm on start_node and generates
						// matrix for start_node
    ~Djikstra() {delete paths;}			// frees allocated matrix space
    void search_path(Graph &G, Queue &Q);	// core of algorithm's search matrix path
    void update_path(int neigh, double offset, vector<double> path);	// updates matrix path with
						// when shortest path is found
    void print();		// prints matrix path
private:
    vector<vector<double>>* paths;	// stores matrix path
    int nodes;
    int from;
};

// Initiates unvisit queue with nodes vector
Queue::Queue(int n) : nodes(n) {
    assert(n>0);
    unvisit = new vector<int>(nodes); 
    assert(unvisit!=0);
    visit = new vector<int>(0); 
    assert(visit!=0);
    for(int i=0; i<nodes; ++i)
        (*unvisit)[i]=i;
}
// Prints visit and unvisit queues
void Queue::print() {
    cout << "Unvisit:" << endl;
    for(int i: (*unvisit))
        cout << setw(3)<< i;
    cout << endl;
    cout << "Visit:" << endl;
    for(int i: (*visit))
        cout << setw(3)<< i;
    cout << endl;
}
// Checks for an element on the unvist queue, returns index location or -1 if not avaliable.
// Checks for empty queue.
int Queue::contains(int element) {
    int location=-1;
    if(!empty()) {
        for(int i=0; i<size(); ++i)
            if((*unvisit)[i] == element)
                location = i;
        return location;
    }
    else 
        return -1;
}
// Removes element on the unvisit queue and puts element on the visit queue
void Queue::remove(int element) {
    int location;
    location = contains(element);
    if(location!=-1) {
        (*unvisit).erase((*unvisit).begin()+location);
        (*visit).push_back(element);
    }
}
// generates a nxn matrix of nodes and assigns random values from 0.0 to 10.0,
// generarted matrix is hollow and symmetrical
Graph::Graph(int n): nodes(n)
{
    assert(n>0);
    matrix= new vector<vector<double>>(nodes,vector<double>(nodes));
    assert(matrix!=0);
    for(int i=0; i<nodes; ++i)
        for(int j=i;j<nodes; ++j)
            if(i == j) (*matrix)[i][j]= 0;
            else (*matrix)[i][j] = (*matrix)[j][i] = rand()%2*\
                (rand()%90/10.0+1.0);
}
// prints whole graph matrix
void Graph::print() {
    double density;
    
    density = 100.0*E()/(nodes*nodes-nodes);
    cout << "Graph:" << endl;
    for(int i=0; i<nodes; ++i) {
        for(int j=0; j<nodes; ++j)
            cout << setw(5) << (*matrix)[i][j];
        cout << endl;
    }
    cout << "Density: " << density << "%"<< endl;
}
// gets number of non-zero edges
int Graph::E() {
    int sum=0;

    for(int i=0; i<nodes; ++i)
        for(int j=0; j<nodes; ++j)
            if((*matrix)[i][j]>0) sum+=1;
    return sum;
}
// returns neighbors for a node path
vector<int> Graph::neighbors(int node) {
    vector<int> a;
    for(int i=0; i<nodes; ++i)
        if((*matrix)[node][i] > 0)
        a.push_back(i);
    return a;
}
// returns node path edge values
vector<double> Graph::get_node_value(int node) {
    vector<double> a;
    for(int i=0; i<nodes; ++i)
        a.push_back((*matrix)[node][i]);
    return a;
}
// set node path edge values
void Graph::set_node_value(int node, vector<double> val) {
    for(int i=0; i<nodes; ++i)
        (*matrix)[node][i] = val[i];
}
// returns edge value for path[node][path]
double Graph::get_edge_value(int node, int path) {
    return((*matrix)[node][path]);
}
// sets edge value for path[node][path]
void Graph::set_edge_value(int node, int path, double val) {
    (*matrix)[node][path] = val;
}
// Allocates space for matrix output path and initializes values for
// start node edges: sets 0 to start_node length and removes node
// from unvisit queue
Djikstra::Djikstra(Graph &G, Queue &Q, int m) : nodes(G.V()), from(m) {
    paths= new vector<vector<double>>(nodes,vector<double>(3));
    vector<double> a;

    a = G.get_node_value(from);
    for(int i=0; i< nodes; ++i) {
        (*paths)[i][0] = i;
        if(a[i] == 0) a[i] = nodes*nodes;
        (*paths)[i][1] = a[i];
        (*paths)[i][2] = from;
    }
    (*paths)[from][1] = 0;
    Q.remove(from);
}
// Searches for paths on the premise of a shortest path
void Djikstra::search_path(Graph &G, Queue &Q) {
    vector<int> neigh;
    vector<double> temp;
    double offset;

    for(int i=0; i < nodes; ++i) {
        neigh = G.neighbors(i);
        for(int j=0; j<neigh.size(); ++j) {
            offset = (*paths)[i][1];
            temp = G.get_node_value(i);
            update_path(i, offset, temp);
            Q.remove(i);
        }
    }
}
// Updates matrix path when short path is founded.
void Djikstra::update_path(int neigh, double offset, vector<double> path) {
    double temp;
    
    for(int i=0; i< nodes; ++i) {
        temp = path[i] + offset;
        if(temp < (*paths)[i][1] && path[i]>0) {
            (*paths)[i][1] = temp;
            (*paths)[i][2] = neigh;
        }
    }
}
// Prints whole output matrix path
void Djikstra::print() {
    cout << "Djikstra:" << endl;
    cout << setw(6) << "node" << setw(6) << "len" << setw(6) << "prev" << endl;
    for(int i=0;i<nodes; ++i) {
        for(int j=0;j<3;++j) {
            if(j==1 && (*paths)[i][j]==nodes*nodes)
                cout << setw(6) << "Inf";
            else
                cout << setw(6) << (*paths)[i][j];
        }
        cout << endl;
    }
}

int main() {
    int nodes, from;
    // seed initializer for random()
    srand(time(NULL));
    // nodes for graph matrix
    cout << "Please enter number of nodes: ";
    cin >> nodes;
    // generates graph matrix
    Graph G(nodes);
    G.print();
    // queues for visit/unvisit nodes
    Queue Q(G.V());
    // perform djikstra's algorithm on node
    cout << "Enter node value to perform Djikstra algorithm: ";
    cin >> from;
    // generates output matrix path
    Djikstra D(G, Q, from);
    D.search_path(G, Q);
    // print results
    D.print();
    Q.print();
}

