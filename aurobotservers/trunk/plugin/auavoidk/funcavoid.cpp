/***************************************************************************
 *   Copyright (C) 2010 by DTU (Kasper Grue Understrup)                    *
 *   jca@oersted.dtu.dk                                                    *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU Lesser General Public License as        *
 *   published by the Free Software Foundation; either version 2 of the    *
 *   License, or (at your option) any later version.                       *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU Lesser General Public License for more details.                   *
 *                                                                         *
 *   You should have received a copy of the GNU Lesser General Public      *
 *   License along with this program; if not, write to the                 *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

/*	TODO
 *
 * */

#include <urob4/uvariable.h>
#include <urob4/uresposehist.h>
#include <urob4/uvarcalc.h>
#include <iostream>

#include "funcavoid.h"

#ifdef LIBRARY_OPEN_NEEDED

///////////////////////////////////////////////////
// library interface
// used by server when function is loaded to create this object
#include <urob4/uvariable.h>

UFunctionBase * createFunc() { // create an object of this type
	//
	/** replace 'Funcavoid' with your classname, as used in the headerfile */
	return new Funcavoid();
}

#endif

///////////////////////////////////////////////////

/* HandleCommand funktionen bliver kørt når plugin-et bliver kørt. Denne funktionen kan derfor blive betragtet som
 * mainfunktionen i mit program. Main funktionen leverer koordinator tilbage til SMRCL-scriptet som skal bruges til
 * obstacle avoidance. Der leveres 7 værdier tilbage l0 er hvor meget robotten skal dreje til at starte, L1 er
 * hvor langt der skal køres frem, l2 er angivet hvis der skal drejes efterfølgende, l3 er itterationsnummeret, som
 * skal bruges af plugin-et til at tjekke om data er leveret tilbage og l4 er 1 hvis robotten er nået frem til målet
 * eller er fejlet undervejs eller 0 hvis den er undervejs.
 * */

bool Funcavoid::handleCommand(UServerInMsg * msg, void * extra) {
	const int MRL = 500;
	char reply[MRL];
	double drej, koer, drejEfter;
	double huller[100][5];
	int randomInt;
	ULaserData * data;
	UResPoseHist * odoPose;
	UTime tid;
	msg->tag.getAttDouble("x", &maal_x);
	msg->tag.getAttDouble("y", &maal_y);
	// check for parameter 'help'
	if (msg->tag.getAttValue("help", NULL, 0)) { // create the reply in XML-like (html - like) format
		sendHelpStart("AVOIDK");
		sendText(
				"--- AVOIDK obstacle avoidance - laser based, primarily to be used from smrcl\n");
		snprintf(
				reply,
				MRL,
				"log=true|false      Opens or closes the logfile %s (open=%s)\n",
				logf.getLogFileName(), bool2str(logf.isOpen()));
		sendText(reply);
		sendText("update              Update variables (default behaviour)\n");
		sendText("reset               Reset variables\n");
		sendText("help       This message\n");
		sendHelpDone();
	}

	else {
		data = getScan(msg, (ULaserData*) extra);
		odoPose = (UResPoseHist *) getStaticResource("odoPose", true);
		if (data->isValid()) {

			//posi = odoPose->getPoseAtTime(data->getScanTime());
			posi = odoPose->getNewest();

			printf("posi.x=%f posi.y=%f posi.h=%f\n", posi.x, posi.y, posi.h);

			//Hvis robotten kan køre direkte til målet, gør den det.
			if (direkte(data, maal_x, maal_y)) {
				printf("Der kan køres direkte til målet\n");
				//lever koordinater tilbage til smrcl

				printf("kør_x_frem=%f kør_y_frem=%f\n", (maal_x - posi.x),
						(maal_y - posi.y));

				drej = findDrej(maal_x, maal_y);
				drejEfter = -posi.h * 180 / M_PI - drej;
				drej = tilPI(drej);
				drejEfter = tilPI(drejEfter);

				koer = hypot((maal_x - posi.x), (maal_y - posi.y));
				faerdig = 1;

				randomInt = rand() % 23;
				snprintf(
						reply,
						MRL,
						"<laser l0=\"%g\" l1=\"%g\" l2=\"%g\" l3=\"%d\"  l4=\"%d\"  l5=\"%g\"  l6=\"%g\" l7=\"%g\" l8=\"%d\"/>\n",
						drej, koer, drejEfter, nummer, faerdig, posi.x, posi.y,
						posi.getHeadingDeg(), randomInt);
				sendMsg(reply);

				//Hvis robotten ikke kan køre direkte til målet prøver robotten at finde alternative ruter.
			} else {
				printf("Der kan IKKE køres direkte til målet\n");
				findSpring(data, huller);
				findHuller(data, huller);
				undersoeg(data, huller, &drej, &koer, &drejEfter);

				drej = tilPI(drej);
				drejEfter = tilPI(drejEfter);

				if (fabs(drej) < 0.5) {
					drej = 0.0;
				}

				snprintf(
						reply,
						MRL,
						"<laser l0=\"%g\" l1=\"%g\" l2=\"%g\" l3=\"%d\"  l4=\"%d\"  l5=\"%g\"  l6=\"%g\" l7=\"%g\"/>\n",
						drej, koer, drejEfter, nummer, faerdig, posi.x, posi.y,
						posi.getHeadingDeg());

				nummer++;
				sendMsg(reply);
			}
		}

		//test();
	}
	return true;
}

////////////////////////////////////////////////////////////////

/* Denne funktion finder spring i laserscannet hvor robotten potentielt kan komme igennem. Dermed ikke sagt at det er muligt.*/
void Funcavoid::findSpring(ULaserData * data, double huller[100][5]) {
	int i, iFoer = 0, antal = 0;
	double rFoer = 0, angleFoer = 0;
	double x1, y1, x2, y2;

	//clear array
	for (i = 0; i < 100; i++) {
		huller[i][0] = '\0';
	}

	iFoer = 0;
	rFoer = data->getRangeMeter(0);
	angleFoer = data->getAngleDeg(0);

	for (i = 1; i < data->getRangeCnt(); i++) {

		x1 = cos(data->getAngleDeg(iFoer) * M_PI / 180.0)
				* data->getRangeMeter(iFoer);
		y1 = sin(data->getAngleDeg(iFoer) * M_PI / 180.0)
				* data->getRangeMeter(iFoer);
		x2 = cos(data->getAngleDeg(i) * M_PI / 180.0) * data->getRangeMeter(i);
		y2 = sin(data->getAngleDeg(i) * M_PI / 180.0) * data->getRangeMeter(i);

		if (data->getRangeMeter(i) > MINRANGE and rFoer > MINRANGE and hypot(x1
				- x2, y1 - y2) > ROBOB) {
			huller[antal][0] = x1;
			huller[antal][1] = y1;
			huller[antal][2] = x2;
			huller[antal++][3] = y2;
			printf("Spring: x1=%f y1=%f x2=%f y2=%f\n", huller[antal - 1][0],
					huller[antal - 1][1], huller[antal - 1][2], huller[antal
							- 1][3]);
		}

		if (data->getRangeMeter(i) > MINRANGE) {
			iFoer = i;
			rFoer = data->getRangeMeter(i);
			angleFoer = data->getAngleDeg(i);
		}
	}
}

////////////////////////////////////////////////////////////////

/* Denne funktion bruger springene fra find spring og finder huller i stedet for. Et hul er det mindst mulige spring
 * fra en kant og til et ikke ved siden af sideliggende punkt*/
void Funcavoid::findHuller(ULaserData * data, double huller[100][5]) {
	int h;
	double x = 0, y = 0;
	bool fundet = false;
	double potHuller[7000][5];

	for (int i = 0; huller[i][0] != '\0'; i++) {

		//clear array
		for (int b = 0; b < 7000; b++) {
			potHuller[b][0] = '\0';
		}
		h = 0;

		fundet = false;
		for (int j = 0; j < data->getRangeCnt(); j++) {

			if (data->getRangeMeter(j) > MINRANGE) {
				x = cos(data->getAngleDeg(j) * M_PI / 180.0)
						* data->getRangeMeter(j);
				y = sin(data->getAngleDeg(j) * M_PI / 180.0)
						* data->getRangeMeter(j);

				if (fundet /*and hypot((huller[i][0] - x), (huller[i][1] - y))
						> ROBOB*/) {
					potHuller[h][0] = huller[i][0];
					potHuller[h][1] = huller[i][1];
					potHuller[h][2] = x;
					potHuller[h][3] = y;
					h++;
					//	printf("h=%d i=%d j=%d x=%f y=%f huller2=%f huller3=%f\n",
					//		h, j, i, potHuller[h - 1][0], potHuller[h - 1][1],
					//		potHuller[h - 1][2], potHuller[h - 1][3]);
				}

				if (x < huller[i][0] + 0.0001 and x > huller[i][0] - 0.0001
						and y < huller[i][1] + 0.0001 and y > huller[i][1]
						- 0.0001) {
					fundet = true;
				}
			}
		}

		fundet = false;
		for (int j = data->getRangeCnt(); j > 0; j--) {
			if (data->getRangeMeter(j) > MINRANGE) {
				x = cos(data->getAngleDeg(j) * M_PI / 180.0)
						* data->getRangeMeter(j);
				y = sin(data->getAngleDeg(j) * M_PI / 180.0)
						* data->getRangeMeter(j);

				if (fundet /*and hypot((huller[i][2] - x), (huller[3][1] - y))
						> ROBOB*/) {
					potHuller[h][0] = x;
					potHuller[h][1] = y;
					potHuller[h][2] = huller[i][2];
					potHuller[h][3] = huller[i][3];
					h++;
					//printf("h=%d i=%d j=%d x=%f y=%f huller2=%f huller3=%f\n",
					//		h, j, i, potHuller[h - 1][0], potHuller[h - 1][1],
					//		potHuller[h - 1][2], potHuller[h - 1][3]);
				}

				if (x < huller[i][2] + 0.0001 and x > huller[i][2] - 0.0001
						and y < huller[i][3] + 0.0001 and y > huller[i][3]
						- 0.0001) {
					fundet = true;
				}
			}

		}

		sorterPotArray(potHuller);
		huller[i][0] = potHuller[0][0];
		huller[i][1] = potHuller[0][1];
		huller[i][2] = potHuller[0][2];
		huller[i][3] = potHuller[0][3];

	}

	/*for (int i = 0; huller[i][0] != '\0'; i++) {
		printf("Huller: x1=%f y1=%f x2=%f y2=%f\n", huller[i][0], huller[i][1],
				huller[i][2], huller[i][3]);
	}*/
}

////////////////////////////////////////////////////////////////

double Funcavoid::tilPI(double tal) {
	while (tal < -180 || tal > 180) {
		if (tal > 180) {
			tal = tal - 360;
		} else if (tal < -180) {
			tal = tal + 360;
		}
	}
	return tal;
}

////////////////////////////////////////////////////////////////

double Funcavoid::findDrej(double x, double y) {
	return (atan2(y - posi.y, x - posi.x) - posi.h) * 180 / M_PI;
}

////////////////////////////////////////////////////////////////

double Funcavoid::findDrejEfter(double x, double y) {
	double vinkel;

	vinkel = atan2(y - posi.y, x - posi.x) * 180 / M_PI;
	return (atan2(maal_y - y, maal_x - x)) * 180 / M_PI - vinkel;
}

////////////////////////////////////////////////////////////////

/* Dette er funktionen som vælger hvilket hul der skal køres til. Dette gør funktionen ved at sorterer arrayet
 * med huller og tage det hul der ligger tættest på målet.*/
void Funcavoid::undersoeg(ULaserData * data, double huller[100][5],
		double * drej, double * koer, double * drejEfter) {
	double x = 0, y = 0;
	double v_hat_x = 0, v_hat_y = 0;
	double x_minus, x_plus;

	//Sorter huller efter midtpunktet som ligger tættest på målet
	sletHuller(huller);
	korrigerHuller(huller);
	sorterArray(huller);


	//Undersøg om robotten kan kører hen foran hullet med en afstand på 35 cm vinkelret på hullet.
	for (int i = 0; huller[i][0] != '\0'; i++) {

		printf("hul nr%d: x1=%f y1=%f x2=%f y2=%f\n", i, huller[i][0],
				huller[i][1], huller[i][2], huller[i][3]);

		v_hat_x = (-1) * (huller[i][3] - huller[i][1]) / hypot((huller[i][2]
				- huller[i][0]), (huller[i][3] - huller[i][1]));
		v_hat_y = (huller[i][2] - huller[i][0]) / hypot((huller[i][2]
				- huller[i][0]), (huller[i][3] - huller[i][1]));

		x = (huller[i][0] + huller[i][2]) / 2;
		y = (huller[i][1] + huller[i][3]) / 2;

		//længden fra robottens position til hullet hhv ene side
		x_plus = hypot((posi.x - (x + v_hat_x * ROBOB)), (posi.y - (y + v_hat_y
				* ROBOB)));
		x_minus = hypot((posi.x - (x - v_hat_x * ROBOB)), (posi.y - (y
				- v_hat_y * ROBOB)));

		if (direkte(data, x+x*ROBOB/hypot(x,y),y+y*ROBOB/hypot(x,y))) {
				*drej = findDrej(x+x*ROBOB/hypot(x,y),y+y*ROBOB/hypot(x,y));
				*koer = hypot(x+x*ROBOB/hypot(x,y)-posi.x,y+y*ROBOB/hypot(x,y)-posi.y);
				*drejEfter = findDrejEfter(x+x*ROBOB/hypot(x,y),y+y*ROBOB/hypot(x,y));

			printf("begge\n");
			faerdig = 0;
			break;
		}

		else if (direkte(data, x + v_hat_x * ROBOB, y + v_hat_y * ROBOB)) {
			*drej = findDrej((x + v_hat_x * ROBOB), (y + v_hat_y * ROBOB));
			*koer = x_plus;
			*drejEfter = findDrejEfter((x + v_hat_x * ROBOB), (y + v_hat_y
					* ROBOB));
			printf("lang\n");
			faerdig = 0;
			break;
		} else if (direkte(data, x - v_hat_x * ROBOB, y - v_hat_y * ROBOB)) {
			*drej = findDrej((x - v_hat_x * ROBOB), (y - v_hat_y * ROBOB));
			printf("x=%f y=%f v_hat_x=%f v_hat_y=%f\n",x,y,v_hat_x,v_hat_y);
			*koer = x_minus;
			*drejEfter = findDrejEfter((x - v_hat_x * ROBOB), (y - v_hat_y
					* ROBOB));
			printf("kort\n");
			faerdig = 0;
			break;
		} else {
			*drej = 0;
			*koer = 0;
			*drejEfter = 0;
			faerdig = 1;
			printf("der kan ikke køres til et af hullerne\n");
		}
	}
	if (huller[0][0] == '\0') {
		*drej = 0;
		*koer = 0;
		*drejEfter = 0;
		faerdig = 1;
		printf("der var ikke nogen huller\n");
	}
}

////////////////////////////////////////////////////////////////

/* Omregner huller fundet med robottens laserscanner til globale koordinator.*/

void Funcavoid::korrigerHuller(double huller[100][5]) {
	int i = 0;
	UPosition pos;

	while (huller[i][0] != '\0') {

		pos = posi.getPoseToMap((huller[i][0] + ROBOW), huller[i][1]);
		huller[i][0] = pos.x;
		huller[i][1] = pos.y;

		pos = posi.getPoseToMap((huller[i][2] + ROBOW), huller[i][3]);
		huller[i][2] = pos.x;
		huller[i][3] = pos.y;
		i++;
	}
}

////////////////////////////////////////////////////////////////

/* Sletter huller som er for små til robotten.*/

void Funcavoid::sletHuller(double huller[100][5]) {
	int i = 0;
	int b = 0;
	while (huller[i][0] != '\0') {
		i++;
	}

	for (int j = 0; j < i; j++) {
		if (hypot(huller[j][2] - huller[j][0], huller[j][3] - huller[j][1])
				< ROBOB) {
			for (b = j; b <= i; b++) {
				huller[b][0] = huller[b + 1][0];
				huller[b][1] = huller[b + 1][1];
				huller[b][2] = huller[b + 1][2];
				huller[b][3] = huller[b + 1][3];
				huller[b][4] = huller[b + 1][4];
			}
			j--;
			i--;
		}
	}

	huller[i][0] = '\0';
	//printf("Efter sortering\n");
	/*for (int i = 0; huller[i][0] != '\0'; i++) {
		printf("Hul%d: x1=%f y1=%f x2=%f y2=%f\n", i, huller[i][0],
				huller[i][1], huller[i][2], huller[i][3]);
	}*/
}

////////////////////////////////////////////////////////////////

/* Dette er en sorteringsalgoritme som først sorterer alle hullernes afstand til målet og dernæst leverer
 * hullerne tilbage i sorteret rækkefølge*/

void Funcavoid::sorterArray(double huller[100][5]) {
	int i = 0, j = 0;
	double iMax[5] = { 0, 0, 0, 0, 0 };
	int index = 0;

	while (huller[i][0] != '\0') {
		huller[i][4] = hypot((maal_x - (huller[i][0] + huller[i][2]) / 2),
				(maal_y - (huller[i][1] + huller[i][3]) / 2));
		i++;
	}

	i = i - 1;

	for (; i > 0; i--) {
		iMax[4] = huller[i][4];
		for (j = i - 1; j >= 0; j--) {
			if (iMax[4] < huller[j][4]) {
				iMax[0] = huller[j][0];
				iMax[1] = huller[j][1];
				iMax[2] = huller[j][2];
				iMax[3] = huller[j][3];
				iMax[4] = huller[j][4];
				index = j;
			}
		}
		if (huller[index][4] > huller[i][4]) {
			huller[index][0] = huller[i][0];
			huller[index][1] = huller[i][1];
			huller[index][2] = huller[i][2];
			huller[index][3] = huller[i][3];
			huller[index][4] = huller[i][4];

			huller[i][0] = iMax[0];
			huller[i][1] = iMax[1];
			huller[i][2] = iMax[2];
			huller[i][3] = iMax[3];
			huller[i][4] = iMax[4];
		}
	}
	for (int i = 0; huller[i][0] != '\0'; i++) {
		printf("Huller Efter sortering: x1=%f y1=%f x2=%f y2=%f afstand=%f\n", huller[i][0], huller[i][1],
				huller[i][2], huller[i][3], huller[i][4]);
	}
}

////////////////////////////////////////////////////////////////

void Funcavoid::sorterPotArray(double huller[100][5]) {
	int i = 0, j = 0;
	double iMax[5] = { 0, 0, 0, 0, 0 };
	int index = 0;

	while (huller[i][0] != '\0') {
		huller[i][4] = hypot(huller[i][0] - huller[i][2], huller[i][1]
				- huller[i][3]);
		i++;
	}

	i = i - 1;

	for (; i > 0; i--) {
		iMax[4] = huller[i][4];
		for (j = i - 1; j >= 0; j--) {
			if (iMax[4] < huller[j][4]) {
				iMax[0] = huller[j][0];
				iMax[1] = huller[j][1];
				iMax[2] = huller[j][2];
				iMax[3] = huller[j][3];
				iMax[4] = huller[j][4];
				index = j;
			}
		}
		if (huller[index][4] > huller[i][4]) {
			huller[index][0] = huller[i][0];
			huller[index][1] = huller[i][1];
			huller[index][2] = huller[i][2];
			huller[index][3] = huller[i][3];
			huller[index][4] = huller[i][4];

			huller[i][0] = iMax[0];
			huller[i][1] = iMax[1];
			huller[i][2] = iMax[2];
			huller[i][3] = iMax[3];
			huller[i][4] = iMax[4];
		}
	}
}

////////////////////////////////////////////////////////////////

/* Denne funktion undersøger om det er muligt for robotten at kører hen til punkt (x,y) uden at stød ind i noget
 * Hvis dette er muligt leverer funktionen true, ellers false*/

bool Funcavoid::direkte(ULaserData * data, double x, double y) { //Scanner om robotten kan køre direkte til punkt.

	double v_x_lokal, v_y_lokal, v_x_global, v_y_global, v_hat_x, v_hat_y,
			katete_x, katete_y;
	double skalarprodukt1, skalarprodukt2;
	int i;
	UPosition pos;

	//normeret hatvektor til vektoren som går fra robottens position til målet
	//Denne vektor ganges med den halve robotbredte
	v_hat_x = (-1) * (y - posi.y) / hypot((x - posi.x), (y - posi.y)) * ROBOB
			/ 2;
	v_hat_y = (x - posi.x) / hypot((x - posi.x), (y - posi.y)) * ROBOB / 2;

	for (i = 0; i < data->getRangeCnt(); i++) {//simulerer 180 graders scan
		if (data->getRangeMeter(i) > MINRANGE) {
			//v_x er en vektor fra robottens position og ud til et punkt i laserscannet
			v_x_lokal = cos(data->getAngleRad(i)) * data->getRangeMeter(i)
					+ ROBOW;
			v_y_lokal = sin(data->getAngleRad(i)) * data->getRangeMeter(i);
			pos = posi.getPoseToMap(v_x_lokal, v_y_lokal);
			katete_x = pos.x - posi.x;
			katete_y = pos.y - posi.y;
			v_x_global = katete_x - v_hat_x;
			v_y_global = katete_y - v_hat_y;

			skalarprodukt1 = v_hat_x * v_x_global + v_hat_y * v_y_global;

			v_x_lokal = cos(data->getAngleRad(i)) * data->getRangeMeter(i)
					+ ROBOW;
			v_y_lokal = sin(data->getAngleRad(i)) * data->getRangeMeter(i);
			pos = posi.getPoseToMap(v_x_lokal, v_y_lokal);
			v_x_global = katete_x + v_hat_x;
			v_y_global = katete_y + v_hat_y;

			skalarprodukt2 = -v_hat_x * v_x_global + -v_hat_y * v_y_global;

			if (skalarprodukt1 < 0 and skalarprodukt2 < 0 and (hypot((x
					- posi.x), (y - posi.y)) + ROBOB) > (hypot(katete_x,
					katete_y))) {
				printf("break i=%d lokal_x=%f lokal_y=%f\n", i, v_x_lokal,
						v_y_lokal);
				return false;
			}
		}
	}
	return true;
}
