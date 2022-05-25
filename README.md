# GAMES101-Assignment06
Ray intersection with the bounding box and completing the BVH search to speed up the intersection.

## Assignment 06

Assignment 6 required ray intersection with the bounding box and completing the BVH search to speed up the intersection.

The most important thing to do is to tease out the rendering flow in the code framework.

First of all, I need to enter the renderer from the **main()**. Generate a ray at each pixel, and traced it. Compared to Assignment 5, the code needs to be changed because of the addition of the class **Ray**.

```CPP
for (int j = 0; j < scene.height; ++j){
    for (int i = 0; i < scene.width; ++i){
    	// generate primary ray direction
        float x = (2*(i+0.5)/(float)scene.width-1)*imageAspectRatio*scale;
        float y = (1-2*(j+0.5)/(float)scene.height)*scale;

		//normalized the direction vector
        Vector3f dir = normalize(Vector3f(x, y, -1)); 

        //create ray in scene
        Ray ray = Ray(eye_pos,dir,0);//o,d,t

        //casting the ray and tracing the ray in scene
        framebuffer[m++] = scene.castRay(ray, 0);
    }
}
```

Within the function **castRay()**, we can find the function **intersect(ray)** for finding the point of intersection between the scene and the ray. And here we can clearly see that it's computing the intersection of BVH with the light.

```CPP
Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}
```

Further down, we can find that BVH is calculating the intersection of the child node with the ray.

```CPP
Intersection BVHAccel::Intersect(const Ray& ray) const
{
    Intersection isect;
    if (!root)
        return isect;
    isect = BVHAccel::getIntersection(root, ray);
    return isect;
}
```

Further, we find that BVH calculates the intersection point of the child node with the ray through the function **getIntersection(root, ray)**. That's the function we need to fill in.

This is a recursive function, and before we can fill this function we need to do the intersection of individual bounding boxes. 


![image](https://user-images.githubusercontent.com/68177870/170245060-67a5b740-ff0b-417a-8427-a5287c46c5a6.png)


A bounding box is an object of the Bound class that has a function **IntersectP()** to determine whether it intersects the light. We need to fill this function first.

```CPP
inline bool Bounds3::IntersectP(const Ray& ray, const Vector3f& invDir,const std::array<int, 3>& dirIsNeg) const
{
    // Calculate t_min(x,y,z) and t_max(x,y,z).
    std::vector<float> t_min,t_max;
    for (int i = 0; i < 3; i++) {
        float tt_min = (pMin[i] - ray.origin[i]) * invDir[i];
        float tt_max = (pMax[i] - ray.origin[i]) * invDir[i];
        if (dirIsNeg[i] < 0) {
            std::swap(tt_min,tt_max);
        }
        t_min.push_back(tt_min);
        t_max.push_back(tt_max);
    }
	
    // Tenter = max{t_min},Texit = {t_max}
    float Tenter = std::max(t_min[0],std::max(t_min[1],t_min[2]));
    float Texit = std::min(t_max[0],std::min(t_max[1],t_max[2]));
	
    //In this case, the bounding box intersects the light.
    if (Tenter < Texit && Texit >= 0) {
        return true;
    }
    return false;
}
```



Now let's go back to the function **getIntersection(root, ray)** to complete the intersection of the ray and the general bounding box. 

![image](https://user-images.githubusercontent.com/68177870/170245089-89719577-86fc-474d-a140-b2bb19d15410.png)


Using Bounding Volume Hierarchy to partition general Bounding boxes. According to the BVH principle, complete this function.

```CPP
Intersection BVHAccel::getIntersection(BVHBuildNode* node, const Ray& ray) const
{
    //Check if the light intersects with the AABB
    Vector3f invDir(1.0f / ray.direction.x, 1.0f / ray.direction.y, ray.direction.z);
    std::array<int, 3> dirIsNeg;
    for (int i = 0; i < 3; i++) {
        dirIsNeg[i] = ray.direction[i] < 0 ? 1 : 0;
    }
    if (!(node->bounds.IntersectP(ray, invDir, dirIsNeg))) {
        //No intersection, return null
        return;
    }

    //If the current AABB is the smallest AABB
    if (node->left == nullptr && node->right == nullptr) {
        //Intersects with the triangle and returns the intersection
        return node->object->getIntersection(ray);
    }
    
    //Recursively find the intersection of sub-bounding boxes
    Intersection hit1 = getIntersection(node->left, ray);//left child node
    Intersection hit2 = getIntersection(node->right, ray);//right child node
	
    //return the closest intersection to origin
    return (hit1.distance < hit2.distance ? hit1 : hit2);
}
```

Next, we need to do the intersection of the light and triangle in the function **getIntersection(ray)**. The implementation is the same as assignment 5.

```CPP
inline Intersection Triangle::getIntersection(Ray ray)
{
    Intersection inter;
    //如果光线方向向量与法线夹角大于90°，不可见
    if (dotProduct(ray.direction, normal) > 0)
        return inter;
    double u, v, t_tmp = 0;//重心坐标和t（[t,b1,b2]）
    Vector3f pvec = crossProduct(ray.direction, e2);//S1 = d x E2
    double det = dotProduct(e1, pvec);// S2= E1*S1
    //分母S1*E1趋近于0，则b1,b2,1-b1-b2趋近无穷大，无交点
    if (fabs(det) < EPSILON) 
        return inter;

    double det_inv = 1. / det; //1/(S1*E1)
    Vector3f tvec = ray.origin - v0;// S = O-P0
    u = dotProduct(tvec, pvec) * det_inv; //b1 =  S1*S/(S1*E1)
    if (u < 0 || u > 1)//b1<0,1-b1-b2<0，无交点
        return inter;

    Vector3f qvec = crossProduct(tvec, e1);//S2 = S x E1
    v = dotProduct(ray.direction, qvec) * det_inv; // b2 = S2*d/(S1*E1)
    if (v < 0 || u + v > 1)//b2<0,1-b1-b2<0，无交点
        return inter;

    t_tmp = dotProduct(e2, qvec) * det_inv;//t = S2*E2/(S1*E1)
    if (t_tmp < 0)
        return inter;
    // TODO find ray triangle intersection
    inter.distance = t_tmp;
    inter.m = m;
    inter.happened = true;
    inter.obj = this;
    inter.normal = normal;
    inter.coords = ray(t_tmp);
    return inter;
}
```

Now, we have completed the basic requirements for this assignment.

Let's take a look at the render result.

![image](https://user-images.githubusercontent.com/68177870/170245123-d32ba113-bcd0-4990-b840-256ef5b9462a.png)

It takes 22s.

![image](https://user-images.githubusercontent.com/68177870/170245146-092d7933-12ba-44bd-a389-a1095518c020.png)

For the improved part, the Surface Area Heuristic method is used to accelerate the BVH.

I will try to implement it after I finish studying GAMES101.
