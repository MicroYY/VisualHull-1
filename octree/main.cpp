#include <iostream>
#include <vector>
#include <cmath>
#include <queue>

using namespace std;

enum Status{UNUSED,BODY,LEAVES};

struct Onode{
    double x,y,z;
    int generation;
    Status s;
    vector<Onode*> childs;
    Onode();
    Onode(double xx,double yy,double zz,int g);
    int sz()const{return childs.size();}
    void SetNode(double xx,double yy,double zz,int g,Status s);
};

Onode::Onode(){
    s=UNUSED;
    childs.resize(8);
    for(int i=0;i<8;i++){
        childs[i]=NULL;
    }
}

//Onode::Onode(double xx,double yy,double zz,int g):Onode(){
//    x=xx;
//    y=zz;
//    z=zz;
//    generation=g;
//};

void Onode::SetNode(double xx,double yy,double zz,int g,Status ss){
    x=xx;
    y=yy;
    z=zz;
    generation=g;
    s=ss;
}

class Octree{
    Onode *onodes;
    long long sz;
    static double x_min,x_max;
    static double y_min,y_max;
    static double z_min,z_max;
public:
    Octree(int g);
    Onode *GetRoot()const{return onodes;}
    template<typename UP> void TravPost(Onode* node_ptr, UP updater);
    void Expand(Onode* node_ptr,Status ss);
    void Print();
};

double Octree::x_min=-5;
double Octree::x_max=5;
double Octree::y_min=-5;
double Octree::y_max=5;
double Octree::z_min=-5;
double Octree::z_max=5;

void Octree::Expand(Onode* node_ptr,Status ss){ //assert:ss!=UNUSED

    const int x_sign[8]={-1,-1,-1,-1,1,1,1,1};
    const int y_sign[8]={-1,-1,1,1,-1,-1,1,1};
    const int z_sign[8]={-1,1,-1,1,-1,1,-1,1};

    double x_step=(x_max-x_min)/4/pow(2,node_ptr->generation);
    double y_step=(y_max-y_min)/4/pow(2,node_ptr->generation);
    double z_step=(z_max-z_min)/4/pow(2,node_ptr->generation);

    for(int i=0;i<8;i++){
        node_ptr->childs[i]=onodes+sz;
        sz++;
        double xx=node_ptr->x+x_sign[i]*x_step;
        double yy=node_ptr->y+y_sign[i]*y_step;
        double zz=node_ptr->z+z_sign[i]*z_step;
        int g=node_ptr->generation+1;
        node_ptr->childs[i]->SetNode(xx,yy,zz,g,ss);

//        cout<<"expand done "<<sz
//        <<" generation: "<<node_ptr->childs[i]->generation
//        <<" coordinates:"<<node_ptr->childs[i]->x
//        <<" "<<node_ptr->childs[i]->y
//        <<" "<<node_ptr->childs[i]->z<<endl;
    }
}

Octree::Octree(int g){      //number of generation.g=0 means only 1 node(root), g=1 means (1+8) nodes, g=2 means (1+8+64) nodes, etc.
    long long target_sz=1;
    for(int i=1;i<g+1;i++){ //calculate the size of a g generation octree
        target_sz+=pow(8,i);
    }
//    cout<<target_sz<<endl;
    onodes=new Onode[target_sz];    //create all objects at one time
    sz=1;
    onodes->SetNode((x_min+x_max)/2,(y_min+y_max)/2,(z_min+z_max)/2,0,BODY);
    queue<Onode*> q_node;
    q_node.push(onodes);
    while(!q_node.empty()){
        Onode* node_ptr=q_node.front();q_node.pop();
        Expand(node_ptr,BODY);
        if(node_ptr->generation<g-1){   //After expanding to the last generation, stop enqueue
            for(int i=0;i<8;i++){
                q_node.push(node_ptr->childs[i]);
            }
        }
    }
}

template<typename UP>
void Octree::TravPost(Onode* node_ptr, UP updater){
    int target_sz=node_ptr->sz();
    for(int i=0;i<target_sz;i++){
        if(node_ptr->childs[i]&&node_ptr->childs[i]->s!=UNUSED){
            TravPost(node_ptr->childs[i],updater);
        }
    }
    updater(node_ptr);
}

void Octree::Print(){
    struct PrintNode{
        void operator()(Onode *node_ptr)const{
            cout<<"generation: "<<node_ptr->generation
            <<" coordinates:"<<node_ptr->x
            <<" "<<node_ptr->y
            <<" "<<node_ptr->z<<endl;
        }
    }updater;
    TravPost(onodes,updater);
}


int main()
{
    Octree test(8);
//    test.Print();
 //   Octree<double>::Expand exp;
//    test.TravPost(test.GetRoot(),exp);
    cout << "Hello world!" << endl;
    return 0;
}
