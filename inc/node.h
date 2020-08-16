#pragma once

struct Node{
    Node(int val, Node* parent, Node* leftChild, Node* rightChild){
        this->val = val;
        this->parent = parent;
        this-> leftChild = leftChild;
        this->rightChild = rightChild;
        this->numofLeafChilds = 0;
    }
    int val;
    int numofLeafChilds;
    Node* parent;
    Node* leftChild;
    Node* rightChild;
};