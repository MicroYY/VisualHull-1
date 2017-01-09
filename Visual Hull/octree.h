#ifndef OCTREE_H_INCLUDED
#define OCTREE_H_INCLUDED

#include <iostream>
#include <vector>
#include <cmath>
#include <queue>

//UNUSED: the point is not available. Default status of a new point
//BODY & BODY_END: when the octree is generated, mark the leaves as BODY_END and the rest as BODY
//BLACK & WHITE: if the node is out of the 3d point cloud, then it's black,otherwise it's white
//MIXED: If a BODY node has and only has black and white child nodes, then it's MIXED
//BODY_BLACK & BODY_WHITE: color for BODY nodes. if a BODY nodes is marked so it means all of its children has the same color
//PIVOT: A node that will support the tree, in its children it's highly likely to find both black and white
//SURFACE: the node is the surface of the point cloud;
enum Node_status{UNUSED,BODY,BODY_END,LEAVES,BLACK,WHITE,MIXED,BODY_BLACK,BODY_WHITE,PIVOT,SURFACE};  //black stands for outside the hull, white stands for inside the hull

struct Onode{
    double x,y,z;
    int generation;
    Node_status status;
    Onode* parent;
    std::vector<Onode*> childs;
    Onode();
	Onode(double xx, double yy, double zz, int g, Node_status ss, Onode * p);
    int num_child()const{return childs.size();}
    void SetNode(double xx,double yy,double zz,int g,Node_status ss,Onode * p);
};

Onode::Onode(){
    childs.resize(8);
    for(int i=0;i<8;i++){
        childs[i]=NULL;
    }
}

Onode::Onode(double xx, double yy, double zz, int g, Node_status ss, Onode * p):Onode(){
	SetNode(xx, yy, zz, g, ss, p);
}

void Onode::SetNode(double xx,double yy,double zz,int g,Node_status ss,Onode * p){
    x=xx;
    y=yy;
    z=zz;
    generation=g;
    status=ss;
    parent=p;
}

class Octree{

	Onode* root_ptr;
    int num_node;

public:

	double x_min, x_max;
	double y_min, y_max;
	double z_min, z_max;

    Octree(int g=0);
	~Octree();
    Onode *GetRoot()const{return root_ptr;}
	Onode *FindPoint(double xx, double yy, double zz,Onode* node)const;
	Node_status GetStatus(double xx, double yy, double zz, Onode* node_ptr)const;
	int GetSize()const { return num_node; }
	void Print();
    template<typename VST> void TravPost(Onode* node_ptr, VST updater);
    void Expand(Onode* node_ptr,Node_status ss);
    Node_status UpdateStatus(Onode* node_ptr);
	void UpdataPivot(Onode* node_ptr);
	
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

Node_status Octree::GetStatus(double xx, double yy, double zz, Onode* node_ptr)const {//assert:all leaves are BLACK or WHITE or SURFACE
	switch (node_ptr->status)
	{
		case(BLACK):case(BODY_BLACK): {	//if it's black or all of its children are black
			return BLACK;
		}
		case(WHITE):case(BODY_WHITE):case(SURFACE):{	//if it's white or surface of all of its children are white
			return WHITE;
		}
		case(PIVOT):case(MIXED):{	//if the color can not be determined by current node
			double dx = xx - node_ptr->x;
			double dy = yy - node_ptr->y;
			double dz = zz - node_ptr->z;
			int rank = (0 < dx) * 4 + (0 < dy) * 2 + (0 < dz);
			return GetStatus(xx, yy, zz, node_ptr->childs[rank]);
		}
		default:
			std::cout << "status of node: " << node_ptr->status << std::endl;	//sentinel
			break;
	}
	return UNUSED;	//sentinel
}

void Octree::Print() {
	struct Printer {
		void operator()(Onode* onode_ptr)const {
			std::cout << onode_ptr->x << ' ' << onode_ptr->y << ' ' << onode_ptr->z << ' '<<(Node_status)onode_ptr->status << std::endl;
		}
	}printer;
	TravPost(root_ptr, printer);
}

void Octree::Expand(Onode* node_ptr,Node_status ss){ //assert:ss!=UNUSED

    const int x_sign[8]={-1,-1,-1,-1,1,1,1,1};
    const int y_sign[8]={-1,-1,1,1,-1,-1,1,1};
    const int z_sign[8]={-1,1,-1,1,-1,1,-1,1};

    double x_step=(x_max-x_min)/4/pow(2,node_ptr->generation);
    double y_step=(y_max-y_min)/4/pow(2,node_ptr->generation);
    double z_step=(z_max-z_min)/4/pow(2,node_ptr->generation);

    for(int i=0;i<8;i++){

		double xx = node_ptr->x + x_sign[i] * x_step;
		double yy = node_ptr->y + y_sign[i] * y_step;
		double zz = node_ptr->z + z_sign[i] * z_step;
		int g = node_ptr->generation + 1;

        node_ptr->childs[i]=new Onode(xx,yy,zz,g,ss,node_ptr);

    }
	num_node += 8;
}

Octree::Octree(int g){      //number of generation.g=0 means only 1 node(root), g=1 means (1+8) nodes, g=2 means (1+8+64) nodes, etc.

	x_min = -5;x_max = 5;
	y_min = -10;y_max = 10;
	z_min = 15;z_max = 30;
	
	root_ptr = new Onode((x_min + x_max) / 2, (y_min + y_max) / 2, (z_min + z_max) / 2, 0, BODY, NULL);//set the root at the center
    num_node=1;

    std::queue<Onode*> q_node;
    q_node.push(root_ptr);
    while(!q_node.empty()){
        Onode* node_ptr=q_node.front();q_node.pop();

        if(node_ptr->generation<g-1){   //Before expanding to the last generation
            Expand(node_ptr,BODY);	//mark the expanded nodes as BODY and
            for(int i=0;i<8;i++){	//enqueue
                q_node.push(node_ptr->childs[i]);
            }
        }else{	//after expanding to the last generation, stop enqueue
            Expand(node_ptr,BODY_END);	//mark as BODY_END
        }
    }
}

Octree::~Octree() {
	struct Destructor {
		void operator()(Onode* node_ptr) {
			delete node_ptr;
		}
	}destructor;
	TravPost(root_ptr, destructor);
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

//all children are black, return black
//all children are white, return while
//some children are black while some children's white, and there's no mixed or body, return mixed
//some children are black while some are white, but childrens are all part of the body(BODY_BLACK||BODY_WHITE), return pivot
//some children are mixed, indicating the whole subtree is mixed and this node is an important pivot, can't return mixed, return pivot
//some children are body, indicating the node has an pivot child, so it's also a pivot, return pivot

Node_status Octree::UpdateStatus(Onode* node_ptr){
	for (int i = 0; i < 8; i++) {	//check all children for mixed or pivot
		if (node_ptr->childs[i]->status==MIXED || node_ptr->childs[i]->status == PIVOT){
			return node_ptr->status = PIVOT;
		}
	}
	for (int i = 0; i < 8; i++) {   //check if it's all black or white
		switch (node_ptr->childs[i]->status){
		case(BLACK):case(BODY_BLACK): {
			switch (node_ptr->status) {
				case(BODY): {
					node_ptr->status = BODY_BLACK;
					break;
				}
				case(BODY_BLACK): {
					break;
				}
				case(BODY_WHITE): {
					if (node_ptr->childs[i]->status == BLACK) {
						return node_ptr->status = MIXED;
					}
					else {
						return node_ptr->status = PIVOT;
					}
					
				}
			}
			break;
		}
		case(WHITE):case(BODY_WHITE): {
			switch (node_ptr->status) {
				case(BODY): {
					node_ptr->status = BODY_WHITE;
					break;
				}
				case(BODY_BLACK): {
					if (node_ptr->childs[i]->status == WHITE) {
						return node_ptr->status = MIXED;
					}
					else {
						return node_ptr->status = PIVOT;
					}
					
				}
				case(BODY_WHITE): {
					break;
				}
			}
			break;
		}
		default:
			return UNUSED;
		}
	}
	return node_ptr->status;
}

void Octree::UpdataPivot(Onode* node_ptr) {
	while (node_ptr != root_ptr && node_ptr->status != PIVOT) {
		node_ptr->status = PIVOT;
		node_ptr = node_ptr->parent;
	}
}

#endif // OCTREE_H_INCLUDED
