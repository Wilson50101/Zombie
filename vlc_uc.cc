#include <ctype.h>
#include <iostream>
#include <fstream>
#include <string>
#include <cassert>
#include <algorithm>

#include "../thesis/device-config.h"
#include "../thesis/parameter-config.h"
#include "../thesis/clustering.h"
#include "../thesis/resource-alloc.h"
#include "../thesis/transmission.h"
#include "../thesis/matrix.h"

#include "ns3/core-module.h"
#include "ns3/applications-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
//#include "ns3/point-to-point-module.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/vlc-channel-helper.h"
#include "ns3/vlc-device-helper.h"
#include "ns3/netanim-module.h"

#include "time.h"
#include <sys/stat.h>

using namespace ns3;

uint16_t servPort = 4000;

vector<double> Received(1, 0);
vector<double> theTime(1, 0);
static const uint32_t writeSize = 2048;
uint8_t data[writeSize];

void TransmitPacket(Ptr<VlcTxNetDevice> tx, Ptr<Packet> p, Ptr<VlcChannel> channel);

void DataTransmission(vector<Cluster> &clusters);
void StartFlow(Ptr<Socket> localSocket, Ipv4Address servAddress,
		uint16_t servPort, Ptr < VlcChannel > channel);
void WriteUntilBufferFull(Ptr<Socket> localSocket, uint32_t txSpace, Ptr<VlcChannel> channel);

static void RxEnd(Ptr<const Packet> p) { // used for tracing and calculating throughput

	//PrintPacketData(p,p->GetSize());

	Received.push_back(Received.back() + p->GetSize()); // appends on the received packet to the received data up until that packet and adds that total to the end of the vector
	theTime.push_back(Simulator::Now().GetSeconds()); // keeps track of the time during simulation that a packet is received
	//NS_LOG_UNCOND("helooooooooooooooooo RxEnd");
}

static void TxEnd(Ptr<const Packet> p) { // also used as a trace and for calculating throughput

	Received.push_back(Received.back() + p->GetSize()); // same as for the RxEnd trace
	theTime.push_back(Simulator::Now().GetSeconds()); 	//
	//NS_LOG_UNCOND("helooooooooooooooooo TxEnd");
}

static void CwndTracer(uint32_t oldval, uint32_t newval) {
	//NS_LOG_INFO("Moving cwnd from " << oldval << " to " << newval);
}

int main(int argc, char *argv[]) {
	//  LogComponentEnable("PacketSink", LOG_LEVEL_ALL);
	srand(time(0));

	CommandLine cmd;
	cmd.Parse(argc, argv);

	ostringstream path;
	path << "/home/erik/workspace/ns-3.25/src/vlc/stats/";

	//***************INPUTS**************//
	double du, da;
	string ue_formation_method, ue_init_method, ap_formation_method;
	string on_off_ap_method = "none";
	double index = 1;
	string vt_ap_iteration_method;
	string interf_calc;
	uint16_t interf_iteration_num;
	uint16_t ue_num;
	double Rmax, fov;

	//enter the number of UEs and Rmax, fov
	cin >> ue_num >> Rmax >> fov;
	path << ue_num << "/";
	mkdir(path.str().c_str(), 0777);
	path << Rmax << "/";
	mkdir(path.str().c_str(), 0777);
	path << fov << "/";
	mkdir(path.str().c_str(), 0777);

	//enter du, da
	cin >> du >> da;

	//enter UE formation method, init seed selection method, AP formation method
	cin >> ue_formation_method >> ue_init_method >> ap_formation_method;

	//create directory
	path << ue_formation_method << "/";
	mkdir(path.str().c_str(), 0777);
	path << ue_init_method << "/";
	mkdir(path.str().c_str(), 0777);
	path << ap_formation_method << "/";
	mkdir(path.str().c_str(), 0777);

	if(ap_formation_method != "centroid" && ap_formation_method != "individual") {

		cin >> on_off_ap_method;
		path << on_off_ap_method << "/";
		mkdir(path.str().c_str(), 0777);

		if(on_off_ap_method == "OffSome" || on_off_ap_method == "OffSome2") {
			cin >> index;
			path << index << "/";
			mkdir(path.str().c_str(), 0777);
		}
	}

	//enter method for VT
	cin >> interf_iteration_num >> vt_ap_iteration_method >> interf_calc;
	path << interf_iteration_num << "/";
	mkdir(path.str().c_str(), 0777);
	path << vt_ap_iteration_method << "/";
	mkdir(path.str().c_str(), 0777);
	path << interf_calc << "/";
	mkdir(path.str().c_str(), 0777);

	//create directory
	path << da << "/";
	mkdir(path.str().c_str(), 0777);
	path << du << ".txt";
	ofstream fout(path.str(), std::ofstream::app);

	cout << path.str() << endl;

	if(!fout) {
		cout << "file not found" << endl;
		return 0;
	}

	//***************INPUTS**************//

	NodeContainer ap, ue;
	ap.Create(ap_num);
	ue.Create(ue_num);

	//Config APs and UEs
	ApConfig(ap, Rmax);
	UeConfig(ue, ue_num, fov);

	double t=0, start, end;

	//Run clustering algorithm
	vector<Cluster> clusters = Clustering(ap, ue, da, du, ue_formation_method,
			ue_init_method, ap_formation_method, on_off_ap_method, index, ClusteringAlgo);

	start = clock();

	//Run RA algorithm
	double ee = ResourceAllocation(clusters, interf_iteration_num, vt_ap_iteration_method,
			interf_calc);
	if(on_off_ap_method == "exhaustive" || on_off_ap_method == "more_exhaustive")
		ee = OnOffAP(clusters, ee, interf_iteration_num, vt_ap_iteration_method,
				interf_calc, on_off_ap_method);

	end = clock();
	t += (end - start) / CLOCKS_PER_SEC * 1000;

	DataTransmission(clusters);

	Simulator::Stop(Seconds(1000.0));
	Simulator::Run();

	//**********RESULTS*************//
	double total_rate, total_power, total_ee = 0, cell_ee, ue_rate, per_ue_ee, estimated_rate;
	uint16_t active_num;

	total_rate = 0, total_power = 0, cell_ee = 0, per_ue_ee = 0, estimated_rate = 0;
	active_num = 0;

	for(vector<Cluster>::iterator cluster = clusters.begin(); cluster != clusters.end();
			++cluster) {
		double cell_rate = 0, cell_power = 0;
		for(vector<Ptr<Node>>::iterator Ue = cluster->UE.begin(); Ue != cluster->UE.end();
				++Ue) {
			ns3::Ptr < VlcRxNetDevice > rx = DynamicCast < VlcRxNetDevice> ((*Ue)->GetDevice(0));

			if(rx->GetTime() > 0 && rx->GetDeviceStatus()){

				double rate = rx->ComputeGoodPut() * 8 / rx->GetTime();

				total_rate += rate;

				cell_rate += rate;

				estimated_rate += 640000 * 8 / rx->GetTime();

			}

			if(rx->GetDeviceStatus())
				active_num += 1;
		}
		for(vector<Ptr<Node>>::iterator Ap = cluster->AP.begin(); Ap != cluster->AP.end();
				++Ap) {
			ns3::Ptr < VlcTxNetDevice > tx = DynamicCast < VlcTxNetDevice> ((*Ap)->GetDevice(0));

			total_power += tx->GetTotalPowerConsumption();
			cell_power += tx->GetTotalPowerConsumption();
		}
		if(cell_power != 0)
			cell_ee += cell_rate / cell_power;
		//per UE EE calculation
		for(vector<Ptr<Node>>::iterator Ue = cluster->UE.begin(); Ue != cluster->UE.end();
				++Ue) {

			uint16_t id = (*Ue)->GetId();
			ns3::Ptr < VlcRxNetDevice > rx = DynamicCast < VlcRxNetDevice> ((*Ue)->GetDevice(0));

			if(rx->GetTime() > 0 && rx->GetDeviceStatus()) {

				double rate = rx->ComputeGoodPut() * 8 / rx->GetTime();

				double power = 0;

				for(vector<Ptr<Node>>::iterator Ap = cluster->AP.begin(); Ap != cluster->AP.end();
						++Ap) {
					ns3::Ptr < VlcTxNetDevice > tx = DynamicCast < VlcTxNetDevice> ((*Ap)->GetDevice(0));
					if(cluster->UE.size() >= 2)
						power += tx->GetVectorPower(id) * pow(tx->GetPrecodingTerm(id), 2);
					else
						power += tx->GetVectorPower(id);
				}
				if(power != 0)
					per_ue_ee += rate / power;
			}
		}
	}

	double variance = 0, avg = 0;

	for(vector<Cluster>::iterator cluster = clusters.begin(); cluster != clusters.end();
			++cluster) {
		avg += cluster->AP.size();
	}
	avg /= clusters.size();
	for(vector<Cluster>::iterator cluster = clusters.begin(); cluster != clusters.end();
			++cluster) {
		variance += pow(cluster->AP.size() - avg, 2);
	}

	if(total_rate != 0) {
		cout << "Estimated per cell EE = " << ee << endl;
		cout << "Estimated data rate = " << estimated_rate / active_num << endl;
		cout << "per cell EE = " << cell_ee / clusters.size() << endl;
		cout << "per UE EE = " << per_ue_ee / active_num << endl;
		cout << "data rate per active UE = " << total_rate / active_num << endl;
		cout << "global EE = " << total_rate / total_power << endl;
		cout << "number of active users = " << active_num << endl;
		cout << "number of clusters = " << clusters.size() << endl;
		cout << "total throughput = " << total_rate << endl;
		cout << "total transmit power = " << total_power << endl;
		cout << "variance of cluster size = " << variance << endl;
		cout << "execution time = " << t << "ms" << endl;

		fout << cell_ee / clusters.size() << " "
				<< per_ue_ee / active_num << " " << total_rate / active_num << " "
				<< total_rate / total_power << " " << active_num << " "
				<< clusters.size() << " " << total_rate << " "
				<< total_power << " " << variance << " " << t << endl;
	}
	else {
		cout << "No transmission" << endl;
		fout << 0 << " " << 0 << " " << 0 << " "
				<< 0 << " " << 0 << " "
				<< 0 << " " << 0 << " "
				<< 0 << " " << 0 << " " << 0 << endl;
	}

	Simulator::Destroy();

	fout.close();

	//**********RESULTS*************//

	return 0;
}


void DataTransmission(vector<Cluster> &clusters) {
	cout << "Transmission starts..." << endl;

	map<string, Ptr<VlcChannel>> channels = ChannelEstablishment(clusters);
	vector<Ipv4InterfaceContainer> ipInterfs = InternetStackInstall(channels);
	vector<Ptr<Socket>> localsockets = ApplicationInstall(channels);

	// initialize the tx buffer.
	for (uint32_t i = 0; i < writeSize; ++i) {
		char m = toascii(97 + i % 26);
		data[i] = m;
	}

	Config::ConnectWithoutContext(
			"/NodeList/0/$ns3::TcpL4Protocol/SocketList/0/CongestionWindow",
			MakeCallback(&CwndTracer));

	for(vector<Cluster>::iterator it = clusters.begin();
						it!=clusters.end(); it++) {
		for(vector<Ptr<Node>>::iterator it_ue = it->UE.begin();
				it_ue != it->UE.end(); ++it_ue) {
			(*it_ue)->TraceConnectWithoutContext("PhyRxEnd", MakeCallback(&RxEnd)); //traces to allow us to see what and when data is sent through the network
			(*it_ue)->TraceConnectWithoutContext("PhyTxEnd", MakeCallback(&TxEnd)); //traces to allow us to see what and when data is received through the network
		}
	}

	vector<Ipv4InterfaceContainer>::iterator i = ipInterfs.begin();
	vector<Ptr<Socket>>::iterator j = localsockets.begin();
	map<string, Ptr<VlcChannel>>::iterator ch = channels.begin();

	for(; i != ipInterfs.end() && j != localsockets.end() && ch != channels.end();
			++i, ++j, ++ch){

		Simulator::Schedule(Seconds(0.0), &StartFlow, *j,
				i->GetAddress(1), servPort, ch->second);
	}

}

void StartFlow(Ptr<Socket> localSocket, Ipv4Address servAddress,
		uint16_t servPort, Ptr < VlcChannel > channel) {

	//NS_LOG_UNCOND("helooooooooooooooooo StartFlow");
	//std::cout << "Enter here: "  << servAddress << std::endl;
	localSocket->Connect(InetSocketAddress(servAddress, servPort)); //connect

	// tell the tcp implementation to call WriteUntilBufferFull again
	// if we blocked and new tx buffer space becomes available
	//localSocket->SetSendCallback(MakeCallback(&WriteUntilBufferFull));
	//WriteUntilBufferFull(localSocket, localSocket->GetTxAvailable(), channel);

	Simulator::Schedule(Seconds(0.0),
			&WriteUntilBufferFull, localSocket, localSocket->GetTxAvailable(),
			channel);
}

void WriteUntilBufferFull(Ptr<Socket> localSocket, uint32_t txSpace, Ptr<VlcChannel> channel) {
	//NS_LOG_UNCOND("helooooooooooooooooo WriteUntilBufferFull");
	const unsigned long totalTxBytes = 640000;

	unsigned long currentTxBytes = 0;
	/*cout << channel->GetDevice(0)->GetNode()->GetId() << " "
			<< channel->GetDevice(1)->GetNode()->GetId() << endl;*/
	while (currentTxBytes < totalTxBytes) {

		uint32_t left = totalTxBytes - currentTxBytes;
		uint32_t dataOffset = currentTxBytes % writeSize;
		/*uint32_t toWrite = writeSize - dataOffset;
		toWrite = std::min (toWrite, left);
		toWrite = std::min (toWrite, localSocket->GetTxAvailable ());*/

		Ptr<Packet> p = Create<Packet>(&data[0], writeSize);
		Ptr<Node> startingNode = localSocket->GetNode();
		Ptr<VlcTxNetDevice> txOne = DynamicCast<VlcTxNetDevice>(startingNode->GetDevice(0) );
		uint16_t rx_id = channel->GetDevice(1)->GetNode()->GetId();

		channel->AccumulatePacketCount();
		//Set transmission time
		double count = (double) channel->GetPacketCount();
		double tx_time = writeSize * 8 / txOne->GetDataRateInbps(rx_id);
		//std::cout << rx_id << " " << txOne->GetDataRateInbps(rx_id) << std::endl;
		//std::cout << startingNode->GetId() << std::endl;
		Time t = Seconds(count * tx_time);
		Simulator::Schedule(t, TransmitPacket, txOne, p, channel);

		//int amountSent = localSocket->Send (&data[0], writeSize, 0);
		/*if(amountSent < 0)
		{
			/*cout << rx_id << endl;
			Ptr<VlcRxNetDevice> rx = DynamicCast<VlcRxNetDevice>(channel->GetDevice(1));
			double rate = rx->ComputeGoodPut() * 8 / tx_time;
			cout << tx_time << endl;*/

			//SubstractInterference
			//txOne->TransmitComplete(Simulator::Now().GetSeconds());*/
			// we will be called again when new tx space becomes available.
			/*return;
		}*/

		currentTxBytes += writeSize;

	}

	//localSocket->Close();
}

void TransmitPacket(Ptr<VlcTxNetDevice> tx, Ptr<Packet> p, Ptr<VlcChannel> channel) {
	//attach channel
	tx->AttachChannel(channel);
	tx->EnqueueDataPacket(p);
}
