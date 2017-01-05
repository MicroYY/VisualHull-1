#include <iostream>
#include <vector>
#include <cmath>

using namespace std;

enum Status{UNUSED,USED};

struct Onode{
    double x,y,z;
    int generation;
    Status s;
    vector<Onode*> childs;
    Onode();
    Onode(double xx,double yy,double zz,int g);
    int sz()const{return childs.size();}
};

Onode::Onode(){
    s=UNUSED;
    childs.resize(8);
//    for(int i=0;i<8;i++){
//        childs[i]=NULL;
//    }
}

Onode::Onode(double xx,double yy,double zz,int g):Onode(){
    x=xx;
    y=zz;
    z=zz;
    generation=g;
};

class Octree{
    Onode *onodes;
    static double x_min,x_max;
    static double y_min,y_max;
    static double z_min,z_max;
public:
    Octree(int g);
    Onode *GetRoot()const{return onodes;}
    template<typename UP> void TravPost(Onode* node_ptr, UP updater);
    friend struct Expand;
};

double Octree::x_min=-5;
double Octree::x_max=5;
double Octree::y_min=-5;
double Octree::y_max=5;
double Octree::z_min=-5;
double Octree::z_max=5;

struct Expand{
    //(0)000 represents x:-1 y:-1 z:-1 (5)101 represents x:1 y:-1 z:1 etc.
    const int x_sign[8]={-1,-1,-1,-1,1,1,1,1};
    const int y_sign[8]={-1,-1,1,1,-1,-1,1,1};
    const int z_sign[8]={-1,1,-1,1,-1,1,-1,1};

    void operator()(Onode *onodes, Onode *node_ptr){   //assert:node_ptr is initialized
        double x_step=(Octree::x_max-Octree::x_min)/4/pow(2,node_ptr->generation);
        double y_step=(Octree::y_max-Octree::y_min)/4/pow(2,node_ptr->generation);
        double z_step=(Octree::z_max-Octree::z_min)/4/pow(2,node_ptr->generation);
        for(int i=0;i<8;i++){
            if(node_ptr->childs[i]){
                continue;
            }else{
                static int j=1;
                node_ptr->childs[i]=onodes+j;
                node_ptr->childs[i]->x=node_ptr->x+x_sign[i]*x_step;
                node_ptr->childs[i]->y=node_ptr->y+y_sign[i]*y_step;
                node_ptr->childs[i]->z=node_ptr->z+z_sign[i]*z_step;
                node_ptr->childs[i]->generation=node_ptr->generation+1;
                node_ptr->childs[i]->s=USED;
//                cout<<"expand done "<<j
//                    <<" generation: "<<node_ptr->childs[i]->generation
//                    <<" coordinates:"<<node_ptr->childs[i]->x
//                    <<" "<<node_ptr->childs[i]->y
//                    <<" "<<node_ptr->childs[i]->z<<endl;
                j++;
            }
        }
    }
};


Octree::Octree(int g){      //number of generation.g=0 means only 1 node(root), g=1 means (1+8) nodes, g=2 means (1+8+64) nodes, etc.
    long long sz=1;
    for(int i=1;i<g+1;i++){
        sz+=pow(8,i);
    }
    cout<<sz<<endl;
    onodes=new Onode[sz];
    onodes->x=(x_min+x_max)/2;
    onodes->y=(y_min+y_max)/2;
    onodes->z=(z_min+z_max)/2;
    onodes->generation=0;
    onodes->s=USED;
    Expand updater;
    while(g--){
        TravPost(onodes,updater);
    }
}

template<typename UP>
void Octree::TravPost(Onode* node_ptr, UP updater){
    int sz=node_ptr->sz();
    for(int i=0;i<sz;i++){
        if(node_ptr->childs[i]&&node_ptr->childs[i]->s==USED){
            TravPost(node_ptr->childs[i],updater);
        }
    }
    updater(onodes,node_ptr);
}


int main()
{
    Octree test(8);
 //   Octree<double>::Expand exp;
//    test.TravPost(test.GetRoot(),exp);
    cout << "Hello world!" << endl;
    return 0;
}
