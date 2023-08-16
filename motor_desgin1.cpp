#include<iostream>
#include<iomanip>
#include<math.h>
#include<conio.h>

using namespace std;

int main(){
    int phase, pole, con, f;
    float synsp, powut, volt,eff, pf, ac, Bav, Kws, DR;
    float Co , Q ;


    std::cout << "Enter the number of phases: ";
    std::cin >> phase;

    std::cout << "Enter the synchronous speed (in rps): ";
    std::cin >> synsp;

    std::cout << "Enter the number of poles: ";
    std::cin >> pole;

    std::cout << "Enter the power output(Kw): ";
    std::cin >> powut;

    std::cout << "Enter the voltage per phase: ";
    std::cin >> volt;

    std::cout << "Enter the frequency: ";
    std::cin >> f;

    std::cout << "Enter the efficiency(in decimal): ";
    std::cin >> eff;

    std::cout << "Enter the power factor: ";
    std::cin >> pf;

    std::cout << "Enter the electric loading: ";
    std::cin >> ac;

    std::cout << "Enter the magnetic loading: ";
    std::cin >> Bav;

    std::cout << "Enter the initial winding factor: ";
    std::cin >> Kws;

    std::cout << "Enter the L/toe design ratio: ";
    std::cin >> DR;

    std::cout << "Enter the connection type (Star-0, Delta-1): ";
    std::cin >> con;
float k1 = 0.001, DsqL, ddd, D, L;
const float PI = 3.1459;

// Main dimensions
Co = 11 * Kws * Bav * ac * k1;
Q = powut / (eff * pf);
DsqL = Q / (Co * synsp);
ddd = (DsqL) * pole / (PI * DR);
D = pow(ddd, 0.33333);
L = DsqL / (D * D);

std::cout << "Stator Inside diameter = " << D << " m" << std::endl;
std::cout << "Gross length = " << L << " m" << std::endl;
std::cout << "Pole pitch = " << L / DR << std::endl;

// Net iron length
int nd;
float Wd, Li;
std::cout << "Enter the number of radial ventilating ducts and width in mm: ";
std::cin >> nd >> Wd;
// Ki is taken as 0.9
Li = 0.9 * (L - nd * Wd * k1);
std::cout << "Net iron length = " << Li << std::endl;
// Stator Design
// Winding

float Es = volt, phim, Ts;
if (con == 0) {
    Es = Es / pow(3, 0.5);
}
phim = Bav * L * (L / DR);  // Flux per pole
Ts = Es / (4.44 * f * phim * Kws); // Stator turns per phase

std::cout << "Flux per pole = " << phim << std::endl;
std::cout << "Stator turns/ph = " << Ts << std::endl;
// Choose Number of Stator slots
float yss;
int q, Ssmax, Ssmin, Ss;

yss = 0.015;
Ssmax = PI * D / yss;

yss = 0.025;
Ssmin = PI * D / yss;

std::cout << "Optimal choices for Ss: ";
q = 1;
Ss = phase * pole * q;

while (Ss < Ssmax) {
    if (Ss > Ssmin) {
        std::cout << Ss << "  ";
    }
    q++;
    Ss = phase * pole * q;
}

std::cout << std::endl;
std::cout << "Enter the designer's choice of the number of stator slots, Ss: ";
std::cin >> Ss;

q = Ss / (phase * pole);
// Conductors
float Zss;

std::cout << "Stator turns/ph = " << Ts << std::endl; 
std::cout << "Total number of conductors = " << 6 * Ts << std::endl; 

Zss = 6 * Ts / Ss;
std::cout << "Stator conductors per slot = " << Zss << std::endl;

std::cout << "Enter the designer's choice of stator turns/phase and conductors/slot (Zs), so that both are integers: ";
std::cin >> Ts >> Zss;
// Calculating Stator winding factor
float ECS, CS, alph, Kp, Kd;

ECS = Ss / pole;
std::cout << "Estimated Coil span = " << ECS << std::endl;

std::cout << "Enter the designer's choice of coil span: ";
std::cin >> CS;

alph = (ECS - CS) * PI / ECS;  // Angle of chording in radians
Kp = cos(alph / 2);
std::cout << "Pitch factor = " << Kp << std::endl;

Kd = sin(PI / 6) / (q * sin(PI / (6 * q)));
std::cout << "Distribution factor = " << Kd << std::endl;

Kws = Kd * Kp;
std::cout << "Stator winding factor = " << Kws << std::endl;
float Scur = Q * pow(10, 3) / (phase * volt);  // Stator current per phase
float cd, cbs, dcon, den, dpcon, wdcon, th;
int typecon;

std::cout << "Enter the chosen value of current-density, amp/mm.sq. (The current density in the stator winding usually lies between 3 to 5 A/mmsq.): ";
std::cin >> cd;

cbs = Scur / cd;  // Conductor- bare section, mm.sq
dcon = pow(4 * cbs / PI, 0.5);
std::cout << "Computed area of the stator conductor = " << cbs << " mm.sq" << std::endl;
std::cout << "Choose the type of conductor: 1 for circular, 2 for rectangular: ";
std::cin >> typecon;

switch (typecon) {
    case 1: {
        std::cout << "Diameter of conductor = " << dcon << " mm" << std::endl;
        std::cout << "Choose the nearest standard diameter: ";
        std::cin >> dcon;
        std::cout << "Standard diameter (mm) = " << dcon << std::endl;
        cbs = PI * dcon * dcon / 4;
        std::cout << "Standard area = " << cbs << " mmsq." << std::endl;
        std::cout << "Current density for stator conductors = " << Scur / cbs << " A/mmsq." << std::endl;
        std::cout << "Enter the diameter of enamelled conductor (mm): ";
        std::cin >> den;
    }
    break;

    case 2: {
        std::cout << "Enter the depth or width of the conductor: ";
        std::cin >> dpcon;
        wdcon = cbs / dpcon;
        std::cout << "Dimensions of the conductor are: " << dpcon << " mm and " << wdcon << " mm" << std::endl;
        std::cout << "Enter the thickness of the covering (mm): ";
        std::cin >> th;
        dpcon = dpcon + 2 * th;
        wdcon = wdcon + 2 * th;
        std::cout << "Dimensions of the insulated conductor are: " << dpcon << " mm and " << wdcon << " mm" << std::endl;
        std::cout << "Area of insulated conductor = " << dpcon * wdcon << " mmsq." << std::endl;
    }
    break;
}
// Slot and tooth dimensions
float MBst, Mwst, Wst, sf, wed, lip, ars, h, DCR, Dss, SwAA, SwB;
cout << endl << "Enter the value of space factor, if not given then put 0: ";
cin >> sf;

cout << endl << "Enter the maximum flux-density in stator tooth (around 1.7 Wb/mmsq.): ";
cin >> MBst;
Mwst = phim / (MBst * Ss * Li / pole);
cout << endl << "Minimum width of stator tooth = " << Mwst * 1000 << " mm";

if (sf != 0) {
    ars = Zss * cbs / sf;
    cout << endl << "Area of each slot = " << ars << " mmsq.";
    cout << endl << "Enter the designer's choice of width of stator tooth, wedge, lip in mm: ";
    cin >> Wst >> wed >> lip;

    SwAA = PI * (D * 1000 + 2 * (wed + lip)) / Ss - Wst;
    DCR = pow(SwAA * SwAA + 4 * PI * ars / Ss, 0.5);
    h = (DCR - SwAA) / (2 * PI / Ss);

    SwB = SwAA + 2 * PI * h / Ss;  // Slot width at bottom
    Dss = h + wed + lip;
    cout << endl << "SwAA = " << SwAA << " DCR = " << DCR << " h = " << h;
    cout << endl << "Depth of slot = " << Dss << " mm     Width of slot at bottom (opposite to lip) = " << SwB << " mm";
} else {
    int Nw, Ndp;
    float Sl, slackw, Sw, slackd, Lmt;
    cout << endl << "Enter the number of wires across the width, thickness of slot insulation, and slack in mm: ";
    cin >> Nw >> Sl >> slackw;

    cout << endl << "Enter the number of wires along the depth, wedge, lip, and slack in mm: ";
    cin >> Ndp >> wed >> lip >> slackd;

    switch (typecon) {
        case 1: {
            Sw = Nw * den + 2 * Sl + slackw;
            Dss = Ndp * den + 3 * Sl + wed + lip + slackd;
        }
        break;
        case 2: {
            Sw = Nw * wdcon + 2 * Sl + slackw;
            Dss = Ndp * dpcon + 3 * Sl + wed + lip + slackd;
        }
        break;
    }

    cout << endl << "Width of slot = " << Sw;
    cout << endl << "Depth of slot = " << Dss;
    Lmt = 2 * L + 2.3 * L / DR + 0.24;
    cout << endl << "Length of mean turn = " << Lmt;
}
// Stator core
float Dsc, Bsc, Do, Dsc1, Bsc1;
cout << endl << "Enter the flux density in the stator core: ";
cin >> Bsc;
Dsc = phim / (2 * Bsc * Li);

cout << endl << "Depth of stator core = " << Dsc << " m";
cout << endl << "Enter the designer's choice of depth of stator core (in mm): ";
cin >> Dsc1;
Bsc1 = Dsc * 1000 * Bsc / Dsc1;
cout << endl << "Flux density in stator core = " << Bsc1 << " Wb/m^2";
Do = D * 1000 + 2 * (Dss + Dsc1);
cout << endl << "Stator outside diameter = " << Do;
// ROTOR DESIGN

// Air gap
float lag, Dr;
lag = 0.2 + 2 * pow(D * L, 0.5);
cout << endl << "Length of air gap = " << lag << " mm";
cout << endl << "Enter the designer's choice of air gap (mm): ";
cin >> lag;
Dr = D * 1000 - 2 * lag;
cout << endl << "Rotor diameter = " << Dr;

// Rotor slots
int Sr;
cout << endl << "Number of stator slots = " << Ss;
cout << endl << "Enter the number of rotor slots: ";
cin >> Sr;

int typr;
cout << endl << "Enter the type of rotor: 0 for squirrel cage, 1 for wound rotor";
cin >> typr;
if (typr == 0) {
    // Rotor Bars
    float Ib, curdenbar, ab, abar, bbar, wrs, dprs, minWt, Lbc, Ls, Lb, Rb, rhob, Lossb;
    int ms = 3; // for 3 phase machine
    Ib = 2 * ms * Kws * Ts * Scur * pf / Sr;
    cout << endl << "Enter the value of current density in rotor bar (A/mmsq.): ";
    cin >> curdenbar;
    ab = Ib / curdenbar;
    cout << endl << "Area of each rotor bar = " << ab << " mmsq.";
    minWt = phim / (Sr * Li * 1.7 / pole);
    cout << endl << "Minimum width of rotor tooth at root = " << minWt;
    cout << endl << "Enter the dimensions of the cross-section of the rotor bar, a and b: ";
    cin >> abar >> bbar;
    ab = abar * bbar;
    cout << endl << "Area of each rotor bar = " << ab << " mm sq.";
    cout << endl << "Enter the width and depth of the rotor slot in mm: ";
    cin >> wrs >> dprs;
    cout << endl << "Enter the length of the bar extended beyond the core and the length increased due to skewing in mm: ";
    cin >> Lbc >> Ls;
    Lb = L * 1000 + 2 * Lbc + Ls;
    cout << endl << "Length of each bar = " << Lb;
    cout << endl << "Enter the value of resistivity of the bar (Ohm mm): ";
    cin >> rhob;
    Rb = rhob * Lb / ab;
    Lossb = Sr * Ib * Ib * Rb;
    cout << endl << "Resistance of copper bar = " << Rb << " ohm";
    cout << endl << "Total copper loss in bar = " << Lossb << " W";
// End Rings
float Ier, curdener, aer, depr, thr, ODer, IDer, MDer, rhoer, Rer, Losser, CuLoss, Fls; // Fls is Full load slip
Ier = Sr * Ib / (PI * pole);
cout << endl << "Enter the value of current density in end rings (A/mmsq.): ";
cin >> curdener;
aer = Ier / curdener;
cout << endl << "Area of the end ring = " << aer << " mmsq.";
cout << endl << "Enter the depth and thickness of the ring in mm: ";
cin >> depr >> thr;
aer = depr * thr;
ODer = Dr - 2 * dprs;
IDer = ODer - 2 * depr;
MDer = (ODer + IDer) / 2;
cout << endl << "Area of the end ring = " << aer << " mmsq.";
cout << endl << "Rotor Outside diameter = " << ODer << " mm";
cout << endl << "Rotor Inside diameter = " << IDer << " mm";
cout << endl << "Rotor Mean diameter = " << MDer << " mm";
cout << endl << "Enter the resistivity of the end ring (Ohm mm): ";
cin >> rhoer;
Rer = rhoer * PI * MDer / aer;
Losser = 2 * Ier * Ier * Rer;
CuLoss = Losser + Lossb;
cout << endl << "Total copper loss in the bar = " << Lossb << " W";
cout << endl << "Total copper loss in the end rings = " << Losser << " W";
cout << endl << "Total copper loss = " << CuLoss << " W";
Fls = CuLoss / (CuLoss + powut * 1000);
cout << endl << "Slip at full load = " << Fls;
// Rotor core
float Dprc, Brc, Di;
Dprc = Dsc1;
Brc = Bsc1;
cout << endl << "Depth of rotor core (same as stator) = " << Dprc << " m";
cout << endl << "Flux density in rotor core (same as stator) = " << Brc;
Di = Dr - 2 * dprs - 2 * Dprc;
cout << endl << "Inner diameter of rotor lamination = " << Di << " mm";
}

return 0;

}