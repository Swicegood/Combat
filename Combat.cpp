
#define OLC_PGE_APPLICATION
#define OLC_PGEX_SOUND
#include "olcPixelGameEngine.h"
// #include "olcPGEX_Sound.h"
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
}
/*
 * Generic function to find if an element of any type exists in list
 */
template <typename T>
bool contains(std::list<T>& listOfElements, const T& element)
{
    // Find the iterator if element in list
    auto it = std::find(listOfElements.begin(), listOfElements.end(), element);
    //return if iterator points to end or not. It points to end then it means element
    // does not exists in list
    return it != listOfElements.end();
}


class Combat : public olc::PixelGameEngine
{
    std::wstring sBoard;
   

    class Tank {

    public:

        Tank() {
            ang = 0;
            bullet_exists = true;
            spinning = false;
            bullet.pos = { 0,0 };
            score = 0;
            tankRect = { {0,0},{8,8},{0,0} };
        }       
        float ang;
        bool bullet_exists;
        bool spinning;
        int score;
        olc::aabb::rect bullet, tankRect;

   };
       

    Tank myTank, otherTank;
    int nBoardWidth;
    int nBoardHeight;
    int nSquareSize;
    bool collision = false;
    bool blocked = false;
    bool spinning = false;
    std::vector<olc::aabb::rect> vRects, vprevtankRects;
    float fAccumTime = 0;
    olc::Sprite* sprTank = nullptr, *sprBG = nullptr, *sprBullet = nullptr, *sprFont = nullptr;
    olc::Decal* decTank = nullptr, * decBG = nullptr, * decBullet = nullptr, *decFont = nullptr;
    int sndIdle = 0, sndDriving = 0, sndPew = 0, sndPow = 0;
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

        sprTank = new olc::Sprite("./assets/tank.png");
        decTank = new olc::Decal(sprTank);
        sprBG = new olc::Sprite("./assets/combat.png");
        decBG = new olc::Decal(sprBG);
        sprBullet = new olc::Sprite("./assets/1pixel.png");
        decBullet = new olc::Decal(sprBullet);
        sprFont = new olc::Sprite("./assets/combat_font.png");
        decFont = new olc::Decal(sprFont);
        
        
      /*  olc::SOUND::InitialiseAudio(44100, 1, 8, 512);
        sndIdle = olc::SOUND::LoadAudioSample("idle.wav");
        sndDriving = olc::SOUND::LoadAudioSample("driving.wav");
        sndPew = olc::SOUND::LoadAudioSample("pew.wav");
        sndPow = olc::SOUND::LoadAudioSample("pow.wav");
        */

        //Initial positiions
        myTank.tankRect.pos = { 70,68 };
        otherTank.tankRect.pos = { 168,68 };
        otherTank.ang = PI;
        
       //Put all Board tiles in vRects
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
        DrawDecal({0, 0}, decBG); //Draw background from GPU

        DrawPartialDecal({ 40,3 }, decFont, { float((myTank.score % 10) * 12), 0 }, { 12,5 }, { 1,1 }, olc::RED);
        if (myTank.score > 9)
            DrawPartialDecal({ 27,3 }, decFont, { float((myTank.score / 10) * 12), 0 }, { 12,5 }, { 1,1 }, olc::RED);

        DrawPartialDecal({132 ,3 }, decFont, { float((otherTank.score % 10) * 12), 0 }, { 12,5 }, { 1,1 }, olc::BLUE);
        if (otherTank.score > 9)
            DrawPartialDecal({ 119,3 }, decFont, { float((otherTank.score / 10) * 12), 0 }, { 12,5 }, { 1,1 }, olc::BLUE);


        if (myTank.spinning) {            
            myTank.ang -= 0.125 * PI;
            if (abs(myTank.ang) >= 2 * PI)
                myTank.ang = 0;
        }
        else {
            // Find first occurence of sample id
          /*  auto s = std::find_if(olc::SOUND::listActiveSamples.begin(), olc::SOUND::listActiveSamples.end(), [&](const olc::SOUND::sCurrentlyPlayingSample& s) { return s.nAudioSampleID == sndIdle; });
            if (s == olc::SOUND::listActiveSamples.end())
                olc::SOUND::PlaySample(sndIdle, true);
                */
            if ((GetKey(olc::UP).bHeld) || (GetKey(olc::DOWN).bHeld)) {
                
                myTank.tankRect.vel = { 6*cosf(r*.125*PI),6*(sinf(r*.125*PI)) };
            }
            else {
                myTank.tankRect.vel = { 0,0 };
             //   olc::SOUND::StopSample(sndDriving);
            }

            if (GetKey(olc::DOWN).bHeld) {
                myTank.tankRect.vel = {-myTank.tankRect.vel.x, -myTank.tankRect.vel.y};
            }

            if (GetKey(olc::LEFT).bHeld) {
                myTank.ang += 0.125 * PI * fElapsedTime * 6;
            }
            if (GetKey(olc::RIGHT).bHeld) {
                myTank.ang -= 0.125 * PI * fElapsedTime * 6;
            }
            if (abs(myTank.ang) >= 2 * PI)
                myTank.ang = 0;

            //Driving sound
        /*    if (myTank.tankRect.vel.x != 0 || myTank.tankRect.vel.y != 0) {
                auto s = std::find_if(olc::SOUND::listActiveSamples.begin(), olc::SOUND::listActiveSamples.end(), [&](const olc::SOUND::sCurrentlyPlayingSample& s) { return s.nAudioSampleID == sndDriving; });
                if (s == olc::SOUND::listActiveSamples.end()) {
                    olc::SOUND::StopSample(sndIdle);
                    olc::SOUND::PlaySample(sndDriving);
                }
                
            }*/
        }

        r = (int((myTank.ang / PI) * 8)); r = (r < 0) ? abs(r) : 16 - r; if (r == 16) r = 0;  //Set rotation to one of 16 positions
        
       

        if (otherTank.spinning) {
            otherTank.ang += 0.125 * PI;
            if (abs(otherTank.ang) >= 2 * PI)
                otherTank.ang = 0;
        }
        else {
           
            if (blocked) {
                otherTank.ang -= (0.125 * PI); 
                blocked = false;
            }
            if (abs(otherTank.ang) >= 2 * PI)
                otherTank.ang = 0;
            q = (int((otherTank.ang / PI) * 8)); q = (q < 0) ? abs(q) : 16 - q; if (q == 16) q = 0;
            otherTank.tankRect.vel = { float(3.0 * cosf(q * 0.125 * PI)),float(3.0 * (sinf(q * 0.125 * PI))) };
            }
        q = (int((otherTank.ang / PI) * 8)); q = (q < 0) ? abs(q) : 16 - q; if (q == 16) q = 0; //Set rotational position
       
       
        //Fire bullet on spacebar
        if ((GetKey(olc::SPACE).bPressed) && myTank.spinning == false && otherTank.spinning == false) {
            myTank.bullet_exists = true;
            myTank.bullet.pos = muzzle_pos[r]+myTank.tankRect.pos;
            myTank.bullet.size = { 1.0,1.0 };
            myTank.bullet.vel = { float(100.0 * cosf(r*0.125*PI)),float (100.0 * (sinf(r*0.125*PI))) };          
          //  olc::SOUND::PlaySample(sndPew);
        }

        fAccumTime += fElapsedTime;
        if (fAccumTime > 1) {
            if (myTank.spinning || otherTank.spinning) {   //Resume after spin
                otherTank.bullet_exists = true;
                myTank.spinning = false; otherTank.spinning = false;
            }
            
        }

        if (fAccumTime >  5) {           
            if (otherTank.bullet_exists) {  //Fire bullet every few seconds
                otherTank.bullet.pos = muzzle_pos[q]+otherTank.tankRect.pos;
                otherTank.bullet.size = { 1.0,1.0 };
                otherTank.bullet.vel = { float(100.0 * cosf(q * 0.125 * PI)),float(100.0 * (sinf(q * 0.125 * PI))) };
               // olc::SOUND::PlaySample(sndPew);
            }  
            fAccumTime = 0;
        }
        
        //Draw Tanks
        
        DrawPartialDecal(myTank.tankRect.pos, decTank, { float((r % 4) * Tanksize),float((r / 4) * Tanksize) }, { float(Tanksize),float(Tanksize) }, { 1, 1 }, olc::RED);
        DrawPartialDecal(otherTank.tankRect.pos, decTank, { float((q % 4) * Tanksize),float((q / 4) * Tanksize) }, { float(Tanksize),float(Tanksize) }, { 1, 1 }, olc::BLUE);

        //Precess motion for tanks and bullets
        for (int k = 0; k < 2; k++) {
            olc::Pixel p, curp;
            Tank* curTank = nullptr;
            Tank* curoppTank = nullptr;

            if (k == 0) {   //Player's tank
                curp = olc::Pixel(255, 0, 0);
                curTank = &myTank;
                curoppTank = &otherTank;

            }
            if (k == 1) { //AI tank
                curp = olc::Pixel(0, 0, 255);
                curTank = &otherTank;
                curoppTank = &myTank;
            }

           
            vRects.push_back(curoppTank->tankRect); //Add Opponent tank to vRects

            olc::vf2d cp, cn;
            float t = 0, min_t = INFINITY;
            std::vector<std::pair<int, float>> z;

            if ((*curTank).bullet_exists) {
                if ((*curTank).bullet.vel.x != 0 || (*curTank).bullet.vel.y != 0) //While bullet in flight
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
                        // Collided with object is opponent tank
                        if (((*curoppTank).tankRect.pos.x == vRects[j.first].pos.x) && ((*curoppTank).tankRect.pos.y == vRects[j.first].pos.y)) {  
                            (*curoppTank).spinning = true; (*curTank).score += 1;
                            if ((*curTank).score > 99)
                                (*curTank).score = 0;
                        //    olc::SOUND::StopSample(sndIdle);                       
                        //    olc::SOUND::PlaySample(sndPow);
                            (*curoppTank).tankRect.vel += (2 * (*curTank).bullet.vel); //Blown back
                            otherTank.bullet_exists = false;  //Pause fighting
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
                    collision = true;
                }
            }
            (*curTank).tankRect.pos += (*curTank).tankRect.vel * fElapsedTime; //Upade position of tank


            if (collision) {
                collision = false;
                if (((*curTank).tankRect.pos.x == otherTank.tankRect.pos.x) && ((*curTank).tankRect.pos.y == otherTank.tankRect.pos.y))
                    blocked = true;
            }

            vRects.pop_back(); //remove opponent tank from vector
        }return true;

    }
     bool OnUserDestroy()
            {
                //olc::SOUND::DestroyAudio();
                return true;
            }
};
 



int main()
{
    Combat game;
    game.Construct(188, 136, 6, 6);
    game.Start();
    return 0;
}