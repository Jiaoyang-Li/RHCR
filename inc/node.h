#pragma once

struct Node{
    Node(int val, Node* parent, Node* leftChild, Node* rightChild){
        this->val = val;
        this->parent = parent;
        this-> leftChild = leftChild;
        this->rightChild = rightChild;
    }
    int val;
    Node* parent;
    Node* leftChild;
    Node* rightChild;
};