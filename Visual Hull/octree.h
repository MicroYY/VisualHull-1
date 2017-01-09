#ifndef OCTREE_H_INCLUDED
#define OCTREE_H_INCLUDED

#include <iostream>
#include <vector>
#include <cmath>
#include <queue>

enum Status{UNUSED,BODY,BODY_END,LEAVES,BLACK,WHITE,MIXED,SURFACE};  //black stands for outside the hull, white stands for inside the hull

struct Onode{
    double x,y,z;
    int generation;
    Status status;
    Onode* parent;
    std::vector<Onode*> childs;
    Onode();
    Onode(double xx,double yy,double zz,int g);
    int num_child()const{return childs.size();}
    void SetNode(double xx,double yy,double zz,int g,Status status,Onode * p);
};

Onode::Onode(){
    status=UNUSED;
    parent=NULL;
    childs.resize(8);
    for(int i=0;i<8;i++){
        childs[i]=NULL;
    }
}

void Onode::SetNode(double xx,double yy,double zz,int g,Status ss,Onode * p){
    x=xx;
    y=yy;
    z=zz;
    generation=g;
    status=ss;
    parent=p;
}

class Octree{

    std::vector<Onode> *v_body_nodes;
	Onode* root_ptr;
    long long num_node;

public:

	double x_min, x_max;
	double y_min, y_max;
	double z_min, z_max;

    Octree(){};
    Octree(int g);
    Onode *GetRoot()const{return root_ptr;}
	Onode *FindPoint(double xx, double yy, double zz,Onode* node)const;
	void Print();
    template<typename VST> void TravPost(Onode* node_ptr, VST updater);
    void Expand(Onode* node_ptr,Status ss);
    Status UpdateStatus(Onode* node_ptr);
	
};

Onode* Octree::FindPoint(double xx, double yy, double zz, Onode* node)const {
	if (node == NULL || node->status == UNUSED) {
		return NULL;
	}
	double dx = xx - node->x ;
	double dy = yy - node->y;
	double dz = zz - node->z;

	if ((abs(dx)<1e-10)&&(abs(dy)<1e-10)&&(abs(dz)<1e-10)) {
		return node;
	}

	int rank = (0 < dx) * 4 + (0 < dy) * 2 + (0 < dz);
	return FindPoint(xx, yy, zz, node->childs[rank]);

}

void Octree::Print() {
	struct Printer {
		
		void operator()(Onode* onode_ptr)const {
			std::cout << onode_ptr->x << ' ' << onode_ptr->y << ' ' << onode_ptr->z << ' '<<(Status)onode_ptr->status << std::endl;
		}
	}printer;
	TravPost(root_ptr, printer);
}

void Octree::Expand(Onode* node_ptr,Status ss){ //assert:ss!=UNUSED

    const int x_sign[8]={-1,-1,-1,-1,1,1,1,1};
    const int y_sign[8]={-1,-1,1,1,-1,-1,1,1};
    const int z_sign[8]={-1,1,-1,1,-1,1,-1,1};

    double x_step=(x_max-x_min)/4/pow(2,node_ptr->generation);
    double y_step=(y_max-y_min)/4/pow(2,node_ptr->generation);
    double z_step=(z_max-z_min)/4/pow(2,node_ptr->generation);

    for(int i=0;i<8;i++){
        switch(ss){
            case(BODY):case(BODY_END):{
                node_ptr->childs[i]=root_ptr+num_node;
                num_node++;
                break;
            }
            case(LEAVES):{
                node_ptr->childs[i]=new Onode;
            }
        }

        double xx=node_ptr->x+x_sign[i]*x_step;
        double yy=node_ptr->y+y_sign[i]*y_step;
        double zz=node_ptr->z+z_sign[i]*z_step;
        int g=node_ptr->generation+1;
        node_ptr->childs[i]->SetNode(xx,yy,zz,g,ss,node_ptr);
    }
}

Octree::Octree(int g){      //number of generation.g=0 means only 1 node(root), g=1 means (1+8) nodes, g=2 means (1+8+64) nodes, etc.

	x_min = -5;x_max = 5;
	y_min = -10;y_max = 10;
	z_min = 15;z_max = 30;

    long long target_sz=1;
    for(int i=1;i<g+1;i++){ //calculate the size of a g generation octree
        target_sz+=pow(8,i);
    }
//    cout<<target_sz<<endl;
	std::vector<Onode> *v_body_nodes=new std::vector<Onode>(target_sz,Onode());    //create all objects at one time
	root_ptr = &(*v_body_nodes)[0];
    num_node=1;
    root_ptr->SetNode((x_min+x_max)/2,(y_min+y_max)/2,(z_min+z_max)/2,0,BODY,NULL);//set the root at the center

    std::queue<Onode*> q_node;
    q_node.push(root_ptr);
    while(!q_node.empty()){
        Onode* node_ptr=q_node.front();q_node.pop();

        if(node_ptr->generation<g-1){   //After expanding to the last generation, stop enqueue
            Expand(node_ptr,BODY);
            for(int i=0;i<8;i++){
                q_node.push(node_ptr->childs[i]);
            }
        }else{
            Expand(node_ptr,BODY_END);
        }
    }
}

template<typename VST>
void Octree::TravPost(Onode* node_ptr, VST updater){
    for(int i=0;i<8;i++){
        if(node_ptr->childs[i]&&node_ptr->childs[i]->status!=UNUSED){
            TravPost(node_ptr->childs[i],updater);
        }
    }
    updater(node_ptr);
}

//all children's black, return black
//all children's white, return while
//some children's black while some children's white, and there's no mixed or body, return mixed
//some children's mixed, indicating the whole subtree is mixed and this node is an important pivot, can't return mixed, return body
//some children's body, indicating the node has an pivot child, so it's also a pivot, return body

Status Octree::UpdateStatus(Onode* node_ptr){
	for (int i = 0; i < 8; i++) {	//check all children for mixed or body
		if (node_ptr->childs[i]->status==MIXED || node_ptr->childs[i]->status == BODY){
			return node_ptr->status = BODY;
		}
	}
	for (int i = 0; i < 8; i++) {   //check if it's all black or white
		switch (node_ptr->childs[i]->status){
		case(BLACK): {
			switch (node_ptr->status) {
			case(BODY): {
				node_ptr->status = BLACK;
				break;
			}
			case(BLACK): {
				break;
			}
			case(WHITE): {
				return node_ptr->status = MIXED;
			}
			}
		}
		case(WHITE): {
			switch (node_ptr->status) {
			case(BODY): {
				node_ptr->status = WHITE;
				break;
			}
			case(BLACK): {
				return node_ptr->status = MIXED;
			}
			case(WHITE): {
				break;
			}
			}
		}
		default:
			break;
		}
	}
	return node_ptr->status;
}

//void Octree::Print(){
//    struct PrintNode{
//        void operator()(Onode *node_ptr)const{
//            cout<<"generation: "<<node_ptr->generation
//            <<" coordinates:"<<node_ptr->x
//            <<" "<<node_ptr->y
//            <<" "<<node_ptr->z<<endl;
//        }
//    }updater;
//    TravPost(onodes,updater);
//}

#endif // OCTREE_H_INCLUDED
