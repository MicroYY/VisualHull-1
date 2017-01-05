#include <iostream>
#include <vector>

using namespace std;

struct Onode{
    double x,y,z;
    int generation;
    vector<Onode*> childs;
    Onode();
    int sz()const{return childs.size();}
};

template<typename T>
Onode<T>::Onode(){
    childs.resize(8);
    for(int i=0;i<8;i++){
        childs[i]=NULL;
    }
}

template<typename T>
class Octree{
    Onode<T> *root;
//    double x_min,x_max;
//    double y_min,y_max;
//    double z_min,z_max;
public:
    Octree(int g);
    Onode<T> *GetRoot()const{return root;}
    template<typename Tup> void TravPost(Onode<T>* node_ptr, Tup updater);
    struct Expand;
};

template<typename T>
Octree<T>::Octree(int g){      //number of generation.g=1 means only 1 node(root), g=2 means (1+8) nodes, g=3 means (1+8+64) nodes, etc.
    root=new Onode<T>;
//    x_min=-5;x_max=5;
//    y_min=-10;y_max=10;
//    z_min=15;z_max=30;
    Expand updater;
    while(g--){
        TravPost(root,updater);
    }
}

template<typename T> template<typename Tup>
void Octree<T>::TravPost(Onode<T>* node_ptr, Tup updater){
    int sz=node_ptr->sz();
    for(int i=0;i<sz;i++){
        if(node_ptr->childs[i]){
            TravPost(node_ptr->childs[i],updater);
        }
    }
    updater(node_ptr);
}

template<typename T>
struct Octree<T>::Expand{
    void operator()(Onode<T> *node_ptr){
        for(int i=0;i<8;i++){
            if(node_ptr->childs[i]){
                continue;
            }else{
                node_ptr->childs[i]=new Onode<T>;
                static int j=0;
                cout<<"expand done "<<j++<<endl;
            }
        }
    }
};

int main()
{
    Octree<double> test(2);
 //   Octree<double>::Expand exp;
//    test.TravPost(test.GetRoot(),exp);
    cout << "Hello world!" << endl;
    return 0;
}
