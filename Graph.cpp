#include "Graph.hpp"

Graph::Graph(vector<vector<int>> adjMat): adjMat(adjMat) {
    int VerticesNum = adjMat.size();
    numEdges = 0;
    for (int i = 0; i < VerticesNum; i++) {
        for(int j=0;j<VerticesNum;j++){
            if(adjMat.at(i).at(i)!=0){
                throw invalid_argument("The numbers on the Diagonal  must be zero");
            }
            if(adjMat.at(i).at(j)!=0){
            numEdges++;
        }
    }
}
}

int Graph::getNumVertices(){
    return VerticesNum;
}
vector<vector<int>> Graph::getAdjMat(){
    return adjMat;
}
void Graph::addEdge(int source, int destiantion, int weight){
    if (source < 0 || destiantion >= VerticesNum || source < 0 || source >= VerticesNum||destiantion==source) {
        throw invalid_argument(" vertex index is Invalid ");
    }
    adjMat.at(source).at(destiantion) = weight;
}
void Graph::removeEdge(int source, int destiantion){
    if (source < 0 || source >= VerticesNum || destiantion < 0 || source >= VerticesNum||destiantion==source) {
        throw invalid_argument(" vertex index is Invalid ");
    }
    adjMat.at(source).at(destiantion) = 0;
}   
