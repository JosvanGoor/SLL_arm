#include "moveit_interface.ih"

void MoveItInterface::initHardCodedPositions()
{
	// Home state
	std::vector<double> position;
	position.push_back(-2.009852409362793);
	position.push_back(-1.6503982543945312);
	position.push_back(0.19134892523288727);
	position.push_back(-1.0986332893371582);
	position.push_back(1.6768240928649902);
	position.push_back(3.232678174972534);
	d_predefined_positions.insert(std::pair<std::string, std::vector<double> >("home", position));

	// Grab high start state, aka Cobra
	position.clear();
	position.push_back(-1.42293480873108);
	position.push_back(-2.3122470378875732);
	position.push_back(-0.844306468963623);
	position.push_back(-1.7411634922027588);
	position.push_back(0.18177272379398346);
	position.push_back(-0.34317547082901);
	d_predefined_positions.insert(std::pair<std::string, std::vector<double> >("cobra", position));

	// Arm right
	position.clear();
	position.push_back(-2.45563440322876);
	position.push_back(-0.6150309443473816);
	position.push_back(-0.8424145579338074);
	position.push_back(-0.6300868988037109);
	position.push_back(0.2381334751844406);
	position.push_back(-0.24043923616409302);
	d_predefined_positions.insert(std::pair<std::string, std::vector<double> >("arm_right", position));

	// Arm left
	position.clear();
	position.push_back(-0.500471410751343);
	position.push_back(-0.4598251283168793);
	position.push_back(-0.33517640829086304);
	position.push_back(1.5199676752090454);
	position.push_back(-1.6397024393081665);
	position.push_back(1.660731315612793);
	d_predefined_positions.insert(std::pair<std::string, std::vector<double> >("arm_left", position));
}
