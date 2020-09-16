
#define OLC_PGE_APPLICATION

#include "olcPixelGameEngine.h"
#include "math.h"
#define PI 3.14159265
#include <algorithm>
#include <functional>
#undef min
#undef max

namespace olc
{
    namespace aabb
    {
        struct rect
        {
            olc::vf2d pos;
            olc::vf2d size;
            olc::vf2d vel;

            std::array<olc::aabb::rect*, 4> contact;
        };

        bool PointVsRect(const olc::vf2d& p, const olc::aabb::rect* r)
        {
            return (p.x >= r->pos.x && p.y >= r->pos.y && p.x < r->pos.x + r->size.x && p.y < r->pos.y + r->size.y);
        }

        bool RectVsRect(const olc::aabb::rect* r1, const olc::aabb::rect* r2)
        {
            return (r1->pos.x < r2->pos.x + r2->size.x && r1->pos.x + r1->size.x > r2->pos.x && r1->pos.y < r2->pos.y + r2->size.y && r1->pos.y + r1->size.y > r2->pos.y);
        }

        bool RayVsRect(const olc::vf2d& ray_origin, const olc::vf2d& ray_dir, const rect* target, olc::vf2d& contact_point, olc::vf2d& contact_normal, float& t_hit_near)
        {
            contact_normal = { 0,0 };
            contact_point = { 0,0 };

            // Cache division
            olc::vf2d invdir = 1.0f / ray_dir;

            // Calculate intersections with rectangle bounding axes
            olc::vf2d t_near = (target->pos - ray_origin) * invdir;
            olc::vf2d t_far = (target->pos + target->size - ray_origin) * invdir;

            if (std::isnan(t_far.y) || std::isnan(t_far.x)) return false;
            if (std::isnan(t_near.y) || std::isnan(t_near.x)) return false;

            // Sort distances
            if (t_near.x > t_far.x) std::swap(t_near.x, t_far.x);
            if (t_near.y > t_far.y) std::swap(t_near.y, t_far.y);

            // Early rejection		
            if (t_near.x > t_far.y || t_near.y > t_far.x) return false;

            // Closest 'time' will be the first contact
            t_hit_near = std::max(t_near.x, t_near.y);

            // Furthest 'time' is contact on opposite side of target
            float t_hit_far = std::min(t_far.x, t_far.y);

            // Reject if ray direction is pointing away from object
            if (t_hit_far < 0)
                return false;

            // Contact point of collision from parametric line equation
            contact_point = ray_origin + t_hit_near * ray_dir;

            if (t_near.x > t_near.y)
                if (invdir.x < 0)
                    contact_normal = { 1, 0 };
                else
                    contact_normal = { -1, 0 };
            else if (t_near.x < t_near.y)
                if (invdir.y < 0)
                    contact_normal = { 0, 1 };
                else
                    contact_normal = { 0, -1 };

            // Note if t_near == t_far, collision is principly in a diagonal
            // so pointless to resolve. By returning a CN={0,0} even though its
            // considered a hit, the resolver wont change anything.
            return true;
        }

        bool DynamicRectVsRect(const olc::aabb::rect* r_dynamic, const float fTimeStep, const olc::aabb::rect& r_static,
            olc::vf2d& contact_point, olc::vf2d& contact_normal, float& contact_time)
        {
            // Check if dynamic rectangle is actually moving - we assume rectangles are NOT in collision to start
            if (r_dynamic->vel.x == 0 && r_dynamic->vel.y == 0)
                return false;

            // Expand target rectangle by source dimensions
            olc::aabb::rect expanded_target;
            expanded_target.pos = r_static.pos - r_dynamic->size / 2;
            expanded_target.size = r_static.size + r_dynamic->size;

            if (RayVsRect(r_dynamic->pos + r_dynamic->size / 2, r_dynamic->vel * fTimeStep, &expanded_target, contact_point, contact_normal, contact_time))
                return (contact_time >= 0.0f && contact_time < 1.0f);
            else
                return false;
        }



        bool ResolveDynamicRectVsRect(olc::aabb::rect* r_dynamic, const float fTimeStep, olc::aabb::rect* r_static)
        {
            olc::vf2d contact_point, contact_normal;
            float contact_time = 0.0f;
            if (DynamicRectVsRect(r_dynamic, fTimeStep, *r_static, contact_point, contact_normal, contact_time))
            {
                if (contact_normal.y > 0) r_dynamic->contact[0] = r_static; else nullptr;
                if (contact_normal.x < 0) r_dynamic->contact[1] = r_static; else nullptr;
                if (contact_normal.y < 0) r_dynamic->contact[2] = r_static; else nullptr;
                if (contact_normal.x > 0) r_dynamic->contact[3] = r_static; else nullptr;

                r_dynamic->vel += contact_normal * olc::vf2d(std::abs(r_dynamic->vel.x), std::abs(r_dynamic->vel.y)) * (1 - contact_time);
                return true;
            }

            return false;
        }
    }
   /* struct matrix3x3
    {
        float m[3][3];
    };

    void Identity(matrix3x3& mat) {

        mat.m[0][0] = 1.0f; mat.m[1][0] = 0.0f; mat.m[2][0] = 0.0f;
        mat.m[0][1] = 0.0f; mat.m[1][1] = 1.0f; mat.m[2][1] = 0.0f;
        mat.m[0][2] = 0.0f; mat.m[1][2] = 0.0f; mat.m[2][2] = 1.0f;
    }


    void Translate(matrix3x3& mat, float ox, float oy) {
        mat.m[0][0] = 1.0f; mat.m[1][0] = 0.0f; mat.m[2][0] = ox;
        mat.m[0][1] = 0.0f; mat.m[1][1] = 1.0f; mat.m[2][1] = oy;
        mat.m[0][2] = 0.0f; mat.m[1][2] = 0.0f; mat.m[2][2] = 1.0f;
    }

    void Rotate(matrix3x3& mat, float fTheta) {
        mat.m[0][0] = cosf(fTheta);  mat.m[1][0] = sinf(fTheta); mat.m[2][0] = 0.0f;
        mat.m[0][1] = -sinf(fTheta); mat.m[1][1] = cosf(fTheta); mat.m[2][1] = 0.0f;
        mat.m[0][2] = 0.0f;          mat.m[1][2] = 0.0f;         mat.m[2][2] = 1.0f;

    }


    void MatixMultiply(matrix3x3& matResult, matrix3x3& matA, matrix3x3& matB) {

        for (int c = 0; c < 3; c++) {
            for (int r = 0; r < 3; r++) {
                matResult.m[c][r] = matA.m[0][r] * matB.m[c][0] +
                    matA.m[1][r] * matB.m[c][1] +
                    matA.m[2][r] * matB.m[c][2];
            }
        }
    }

    void Forward(matrix3x3& mat, float in_x, float in_y, float& out_x, float& out_y) {
        out_x = in_x * mat.m[0][0] + in_y * mat.m[1][0] + mat.m[2][0];
        out_y = in_x * mat.m[0][1] + in_y * mat.m[1][1] + mat.m[2][1];


    }

    void Invert(matrix3x3& matIn, matrix3x3& matOut)
    {
        float det = matIn.m[0][0] * (matIn.m[1][1] * matIn.m[2][2] - matIn.m[1][2] * matIn.m[2][1]) -
            matIn.m[1][0] * (matIn.m[0][1] * matIn.m[2][2] - matIn.m[2][1] * matIn.m[0][2]) +
            matIn.m[2][0] * (matIn.m[0][1] * matIn.m[1][2] - matIn.m[1][1] * matIn.m[0][2]);

        float idet = 1.0f / det;
        matOut.m[0][0] = (matIn.m[1][1] * matIn.m[2][2] - matIn.m[1][2] * matIn.m[2][1]) * idet;
        matOut.m[1][0] = (matIn.m[2][0] * matIn.m[1][2] - matIn.m[1][0] * matIn.m[2][2]) * idet;
        matOut.m[2][0] = (matIn.m[1][0] * matIn.m[2][1] - matIn.m[2][0] * matIn.m[1][1]) * idet;
        matOut.m[0][1] = (matIn.m[2][1] * matIn.m[0][2] - matIn.m[0][1] * matIn.m[2][2]) * idet;
        matOut.m[1][1] = (matIn.m[0][0] * matIn.m[2][2] - matIn.m[2][0] * matIn.m[0][2]) * idet;
        matOut.m[2][1] = (matIn.m[0][1] * matIn.m[2][0] - matIn.m[0][0] * matIn.m[2][1]) * idet;
        matOut.m[0][2] = (matIn.m[0][1] * matIn.m[1][2] - matIn.m[0][2] * matIn.m[1][1]) * idet;
        matOut.m[1][2] = (matIn.m[0][2] * matIn.m[1][0] - matIn.m[0][0] * matIn.m[1][2]) * idet;
        matOut.m[2][2] = (matIn.m[0][0] * matIn.m[1][1] - matIn.m[0][1] * matIn.m[1][0]) * idet;
    }*/

}

class Combat : public olc::PixelGameEngine
{
    std::wstring sBoard;
   /* std::wstring sTank;*/
   

    class Tank {

    public:

        Tank() {
            ang = 0;
            bullet_exists = true;
            spinning = false;
            bullet.pos = { 0,0 };
            tankRect = { {0,0},{8,8},{0,0} };
        }       
        float ang;
        bool bullet_exists;
        bool spinning;
        olc::aabb::rect bullet, tankRect;

   };
       

    Tank myTank, otherTank, prevTank, prevOtherTank;
    int nBoardWidth;
    int nBoardHeight;
    int nSquareSize;
    bool collision = false;
    bool blocked = false;
    bool draw = true;
    bool spinning = false;
    std::vector<olc::aabb::rect> vRects, vprevtankRects;
    float fAccumTime = 0;
    olc::Sprite* sprTank = nullptr, *sprBG = nullptr, *sprBullet = nullptr;
    olc::Decal* decTank = nullptr, *decBG = nullptr, *decBullet = nullptr;
    int r = 0,q = 8;
    int Tanksize = 8;
    olc::vf2d muzzle_pos[16] = { {7,3} ,{ 7,5 }, { 7,7 }, { 5,7 },{ 3,7 }, {2,7} ,{ 0,7 }, { 0,5 }, { 0,3 }, { 0,2 }, { 0,0 }, { 2,0 }, { 3,0 }, { 5,0 }, { 7,0 }, { 7,2 }, };
    

    virtual bool OnUserCreate()
    {
        nBoardWidth = 47;
        nBoardHeight = 34;
        nSquareSize = 4;

        sBoard += L"...............................................";
        sBoard += L"...............................................";
        sBoard += L"...............................................";
        sBoard += L"###############################################";
        sBoard += L"#.....................###.....................#";
        sBoard += L"#.....................###.....................#";
        sBoard += L"#.....................###.....................#";
        sBoard += L"#.............................................#";
        sBoard += L"#......####.........................####......#";
        sBoard += L"#.............................................#";
        sBoard += L"#.............................................#";
        sBoard += L"#...............####.......####...............#";
        sBoard += L"#...............##...........##...............#";
        sBoard += L"#.....##...............................##.....#";
        sBoard += L"#......#...............................#......#";
        sBoard += L"#......#...............................#......#";
        sBoard += L"#......#...............................#......#";
        sBoard += L"#......#....##...................##....#......#";
        sBoard += L"#......#....##...................##....#......#";
        sBoard += L"#......#....##...................##....#......#";
        sBoard += L"#......#...............................#......#";
        sBoard += L"#......#...............................#......#";
        sBoard += L"#.....##...............................##.....#";
        sBoard += L"#.............................................#";
        sBoard += L"#...............##...........##...............#";
        sBoard += L"#...............####.......####...............#";
        sBoard += L"#.............................................#";
        sBoard += L"#.............................................#";
        sBoard += L"#......####.........................####......#";
        sBoard += L"#.............................................#";
        sBoard += L"#.....................###.....................#";
        sBoard += L"#.....................###.....................#";
        sBoard += L"#.....................###.....................#";
        sBoard += L"###############################################";

        sprTank = new olc::Sprite("tank.png");
        decTank = new olc::Decal(sprTank);
        sprBG = new olc::Sprite("combat.png");
        decBG = new olc::Decal(sprBG);
        sprBullet = new olc::Sprite("1pixel.png");
        decBullet = new olc::Decal(sprBullet);


        /*sTank += L"#######..";
        sTank += L"#######..";
        sTank += L"..###....";
        sTank += L"..#######";
        sTank += L"..###....";
        sTank += L"#######..";
        sTank += L"#######..";*/

  
        
       /* myTank.xpos = 70;
        myTank.ypos = 68;
        otherTank.xpos = 140;
        otherTank.ypos = 68;*/
        myTank.tankRect.pos = { 70,68 };
        otherTank.tankRect.pos = { 168,68 };
        otherTank.ang = PI;
        
       
       for (int y = 0; y < nBoardHeight; y++) {
           for (int x = 0; x < (nBoardWidth); x++) {
                if (sBoard[(y * nBoardWidth) + x] == '#') {
                    vRects.push_back({ {float (x * nSquareSize),float( y * nSquareSize)}, {float (nSquareSize), float (nSquareSize)} });
                }
               
            }

        }
        

        return true;
    }



    virtual bool OnUserUpdate(float fElapsedTime)
    {
        /*  olc::matrix3x3 matFinal, matA, matB, matC, matFinalInv;
        olc::matrix3x3 mat2Final, mat2A, mat2B, mat2C, mat2FinalInv;
        olc::matrix3x3 curmatFinal, curmatFinalInv, curomatFinalInv;*/

        /* Clear(olc::Pixel(136, 140, 137));*/


         /*for (int x = 0; x < (nBoardWidth); x++) {
             for (int y = 0; y < nBoardHeight; y++) {
                 if (sBoard[(y * nBoardWidth) + x] == '#') {
                     FillRect(x * nSquareSize, y * nSquareSize, ((x + 1) * nSquareSize), (((y + 1) * nSquareSize)), olc::Pixel(145, 44, 35));
                 }
                 else {
                     FillRect(x * nSquareSize, y * nSquareSize, ((x + 1) * nSquareSize), ((y + 1) * nSquareSize), olc::Pixel(136, 140, 137));

                 }
             }

         }*/

        DrawDecal({0, 0}, decBG);

        prevTank = myTank;
        prevOtherTank = otherTank;

        // Handle Input
        // Handle Player Input
        if (myTank.spinning) {            
           /* r--; if (r == -1) { r = 15; }*/
            myTank.ang -= 0.125 * PI;
            if (abs(myTank.ang) >= 2 * PI)
                myTank.ang = 0;
        }
        else {
            if ((GetKey(olc::UP).bHeld) || (GetKey(olc::DOWN).bHeld)) {
                
                myTank.tankRect.vel = { 6*cosf(r*.125*PI),6*(sinf(r*.125*PI)) };
            }
            else {
                myTank.tankRect.vel = { 0,0 };
            }

            if (GetKey(olc::DOWN).bHeld) {
                myTank.tankRect.vel = {-myTank.tankRect.vel.x, -myTank.tankRect.vel.y};
                /*myTank.vx = -myTank.vx; myTank.vy = -myTank.vy;*/
            }

            if (GetKey(olc::LEFT).bHeld) {
                /*r++; if (r == 16)  r = 0;*/
                myTank.ang += 0.125 * PI * fElapsedTime * 6;
            }
            if (GetKey(olc::RIGHT).bHeld) {
                /*r--; if (r == -1) r = 15;*/
                myTank.ang -= 0.125 * PI * fElapsedTime * 6;
            }
            if (abs(myTank.ang) >= 2 * PI)
                myTank.ang = 0;
           
           /* myTank.xpos += myTank.vx; myTank.ypos += myTank.vy;*/
        }

        r = (int((myTank.ang / PI) * 8)); r = (r < 0) ? abs(r) : 16 - r; if (r == 16) r = 0;
        
        

        if (otherTank.spinning) {
            /*q--; if (q == -1) { q = 15; }*/
            otherTank.ang += 0.125 * PI;
            if (abs(otherTank.ang) >= 2 * PI)
                otherTank.ang = 0;
        }
        else {
           
            if (blocked) {
                /*q--; if (q == -1) { q = 15; }*/
                otherTank.ang -= (0.125 * PI); 
               /* otherTank.xpos -= 2 * otherTank.vx; otherTank.ypos -= 2 * otherTank.vy;*/
               /* otherTank.tankRect.vel = -2 * otherTank.tankRect.vel;*/
                blocked = false;
            }
            if (abs(otherTank.ang) >= 2 * PI)
                otherTank.ang = 0;
            q = (int((otherTank.ang / PI) * 8)); q = (q < 0) ? abs(q) : 16 - q; if (q == 16) q = 0;
            otherTank.tankRect.vel = { float(3.0 * cosf(q * 0.125 * PI)),float(3.0 * (sinf(q * 0.125 * PI))) };
            }
        q = (int((otherTank.ang / PI) * 8)); q = (q < 0) ? abs(q) : 16 - q; if (q == 16) q = 0;
       
        

        //olc::Translate(matA, -4, -3);
        //olc::Translate(mat2A, -4, -3);
        //olc::Rotate(matB, myTank.ang);
        //olc::Rotate(mat2B, otherTank.ang);
        //olc::MatixMultiply(matC, matB, matA);
        //olc::MatixMultiply(mat2C, mat2B, mat2A);
        //olc::Translate(matA, myTank.xpos+4, myTank.ypos+3);
        //olc::Translate(mat2A, otherTank.xpos + 4, otherTank.ypos + 3);
        //olc::MatixMultiply(matFinal, matA, matC);
        //olc::MatixMultiply(mat2Final, mat2A, mat2C);
        //    
        //olc::Invert(matFinal, matFinalInv);
        //olc::Invert(mat2Final, mat2FinalInv);

       

        if ((GetKey(olc::SPACE).bPressed) && myTank.spinning == false && otherTank.spinning == false) {
            myTank.bullet_exists = true;
        /*    olc::Forward(matFinal, 8, 3, myTank.bullet.pos.x, myTank.bullet.pos.y);*/
            myTank.bullet.pos = muzzle_pos[r]+myTank.tankRect.pos;
            myTank.bullet.size = { 1.0,1.0 };
            myTank.bullet.vel = { float(100.0 * cosf(r*0.125*PI)),float (100.0 * (sinf(r*0.125*PI))) };          

        }

        fAccumTime += fElapsedTime;
        if (fAccumTime >  5) {
            if (myTank.spinning || otherTank.spinning) {   //Resume after spin
                otherTank.bullet_exists = true;
                myTank.spinning = false; otherTank.spinning = false;
            }
            if (otherTank.bullet_exists) {
             /*   olc::Forward(mat2Final, 8, 3, otherTank.bullet.pos.x, otherTank.bullet.pos.y);*/
                otherTank.bullet.pos = muzzle_pos[q]+otherTank.tankRect.pos;
                otherTank.bullet.size = { 1.0,1.0 };
                otherTank.bullet.vel = { float(100.0 * cosf(q * 0.125 * PI)),float(100.0 * (sinf(q * 0.125 * PI))) };                
            }  
            fAccumTime = 0;
        }
        
        //Draw Tanks
        
        DrawPartialDecal(myTank.tankRect.pos, decTank, { float((r % 4) * Tanksize),float((r / 4) * Tanksize) }, { float(Tanksize),float(Tanksize) }, { 1, 1 }, olc::RED);
        DrawPartialDecal(otherTank.tankRect.pos, decTank, { float((q % 4) * Tanksize),float((q / 4) * Tanksize) }, { float(Tanksize),float(Tanksize) }, { 1, 1 }, olc::BLUE);




        //Resume after spin


        
       

        for (int k = 0; k < 2; k++) {
            olc::Pixel p, curp;
            Tank* curTank = nullptr;
            Tank lastTank = prevTank;
            Tank* curoppTank = nullptr;

            if (k == 0) {
                /* curmatFinal = matFinal;
                 curmatFinalInv = matFinalInv;
                 curomatFinalInv = mat2FinalInv;*/
                curp = olc::Pixel(255, 0, 0);
                curTank = &myTank;
                lastTank = prevTank;
                curoppTank = &otherTank;

            }
            if (k == 1) {
                /*curmatFinal = mat2Final;
                curmatFinalInv = mat2FinalInv;
                curomatFinalInv = matFinalInv;*/
                curp = olc::Pixel(0, 0, 255);
                curTank = &otherTank;
                curoppTank = &myTank;
                lastTank = prevOtherTank;
            }

            // Work out bounding box of sprite post-transformation
            // by passing through sprite corner locations into 
            // transformation matrix

    //        float ex, ey;
    //        float sx, sy;
    //        float px, py;

    //            olc::Forward(curmatFinal, 0.0f, 0.0f, px, py);
    //            sx = px; sy = py;
    //            ex = px; ey = py;

    //            olc::Forward(curmatFinal, 9.0f, 7.0f, px, py);
    //            sx = std::min(sx, px); sy = std::min(sy, py);
    //            ex = std::max(ex, px); ey = std::max(ey, py);

    //            olc::Forward(curmatFinal, 0.0f, 7.0f, px, py);
    //            sx = std::min(sx, px); sy = std::min(sy, py);
    //            ex = std::max(ex, px); ey = std::max(ey, py);

    //            olc::Forward(curmatFinal, 9.0f, 0.0f, px, py);
    //            sx = std::min(sx, px); sy = std::min(sy, py);
    //            ex = std::max(ex, px); ey = std::max(ey, py);

    //        // Use transformed corner locations in screen space to establish
    //        // region of pixels to fill, using inverse transform to sample
    //        // sprite at suitable locations.

    //        std::vector<olc::aabb::rect> vcurtankRects;
    //        for (int x = sx; x < ex; x++)
    //        {
    //            for (int y = sy; y < ey; y++)
    //            {
    //                float nx, ny, ox, oy;
    //                olc::Forward(curmatFinalInv, (float)x, (float)y, nx, ny);
    //                olc::Forward(curomatFinalInv,(float)x, (float)y, ox, oy);
    //                p = olc::Pixel(136, 140, 137);
    //                if ((0 <= nx && nx <= 8) && (0 <= ny && ny <= 6)) {
    //                    if (sTank[((int32_t)(ny + 0.5f) * 9) + (int32_t)(nx + 0.5f)] == '#') {
    //                        p = curp;
    //                        vcurtankRects.push_back({ {float(x),float(y)}, {1,1} });
    //                        if (sBoard[(floor(y / 4) * nBoardWidth + floor(x / 4))] == '#')  //detect collition
    //                            collision = true;
    //                        if ((0 <= ox && ox <= 8) && (0 <= oy && oy <= 6)) {
    //                            if (sTank[((int32_t)(oy + 0.5f) * 9) + (int32_t)(ox + 0.5f)] == '#')  //detect collision
    //                                collision = true;
    //                        }
    //                    }
    //                }
    //                else {
    //                        if ((0 <= ox && ox <= 8) && (0 <= oy && oy <= 6)) {
    //                            if (sTank[((int32_t)(oy + 0.5f) * 9) + (int32_t)(ox + 0.5f)] == '#')
    //                                draw = false;

    //                        }
    //                }

    //                if ((sBoard[(floor(y / 4) * nBoardWidth + floor(x / 4))] != '#') && draw == true) {
    //                    Draw(x, y, p);
    //                    
    //                }
    //                draw = true;
    //            }
    //        }

    //        

    //        //concat tank rect vector with board rectangle vector
          /*  int m = vprevtankRects.size();

            for (size_t i = 0; i < m ; i++) {*/
            //olc::aabb::rect curTankRect({ { float(curoppTank->xpos),float(curoppTank->ypos) }, { float(Tanksize),float(Tanksize) },{float((*curTank).vx), float((*curTank).vy)} });
            vRects.push_back(curoppTank->tankRect);
            //        }
            //                 
            
                    // Sort collisions in order of distance
            olc::vf2d cp, cn;
            float t = 0, min_t = INFINITY;
            std::vector<std::pair<int, float>> z;

            if ((*curTank).bullet_exists) {
                if ((*curTank).bullet.vel.x != 0 || (*curTank).bullet.vel.y != 0)
                    DrawDecal((*curTank).bullet.pos,decBullet) ;
                // Work out collision point, add it to vector along with rect ID
                for (size_t i = 0; i < vRects.size(); i++)
                {
                    if (olc::aabb::DynamicRectVsRect(&(*curTank).bullet, fElapsedTime, vRects[i], cp, cn, t))
                    {
                        z.push_back({ i, t });
                    }
                }

                // Do the sort
                std::sort(z.begin(), z.end(), [](const std::pair<int, float>& a, const std::pair<int, float>& b)
                    {
                        return a.second < b.second;
                    });

                // Now resolve the collision in correct order 
                for (auto j : z) {
                    if (olc::aabb::ResolveDynamicRectVsRect(&(*curTank).bullet, fElapsedTime, &vRects[j.first])) {
                        //    if vRects[j.first] is in vprevtankRects then spin around
                        //for (int l = 0; l < vprevtankRects.size(); l++) {
                        if (((*curoppTank).tankRect.pos.x == vRects[j.first].pos.x) && ((*curoppTank).tankRect.pos.y == vRects[j.first].pos.y)) {

                        (*curoppTank).spinning = true;
                        /*(*curoppTank).xpos += int(0.5 * (*curTank).bullet.vel.x);
                        (*curoppTank).ypos += int(0.5 * (*curTank).bullet.vel.y);*/
                        (*curoppTank).tankRect.vel += (2 * (*curTank).bullet.vel);
                        otherTank.bullet_exists = false;
                        fAccumTime = 0;                        
                        }
                        (*curTank).bullet.vel = { 0,0 };
                     }
                    
                }
                // UPdate the bullet rectangles position, with its modified velocity
                (*curTank).bullet.pos += (*curTank).bullet.vel * fElapsedTime;
            }
            olc::vf2d dp, dn;
            t = 0, min_t = INFINITY;
            std::vector<std::pair<int, float>> zz;


            // Work out collision point, add it to vector along with rect ID
            for (size_t i = 0; i < vRects.size(); i++)
            {
                if (olc::aabb::DynamicRectVsRect(&(*curTank).tankRect, fElapsedTime, vRects[i], cp, cn, t))
                {
                    zz.push_back({ i, t });
                }
            }

            // Do the sort
            std::sort(zz.begin(), zz.end(), [](const std::pair<int, float>& a, const std::pair<int, float>& b)
                {
                    return a.second < b.second;
                });

            // Now resolve the collision in correct order 
            for (auto j : zz) {
                if (olc::aabb::ResolveDynamicRectVsRect(&(*curTank).tankRect, fElapsedTime, &vRects[j.first])) {
                    //    if vRects[j.first] is in vprevtankRects then spin around
                    //for (int l = 0; l < vprevtankRects.size(); l++) {
                    //    if (vprevtankRects[l].pos.x == vRects[j.first].pos.x) {
                    //        if (vprevtankRects[l].pos.y == vRects[j.first].pos.y) {
                   /* (*curoppTank).spinning = true;
                    (*curoppTank).xpos += int(0.5 * (*curTank).bullet.vel.x);
                    (*curoppTank).ypos += int(0.5 * (*curTank).bullet.vel.y);*/
                    /*otherTank.bullet_exists = false;
                    fAccumTime = 0;*/
                    /*     }
                     }
                 }*/
                    collision = true;
                }
            }
            (*curTank).tankRect.pos += (*curTank).tankRect.vel * fElapsedTime;



            if (collision) {
               /* (*curTank) = lastTank;*/
                collision = false;
                if (((*curTank).tankRect.pos.x == otherTank.tankRect.pos.x) && ((*curTank).tankRect.pos.y == otherTank.tankRect.pos.y))
                    blocked = true;
            }

            /*for (size_t i = 0; i < m; i++) {*/
            vRects.pop_back();
            //}

            /*vprevtankRects = vcurtankRects;*/
        }return true;
    }

};
 



int main()
{
    Combat game;
    game.Construct(188, 136, 6, 6);
    game.Start();
    return 0;
}