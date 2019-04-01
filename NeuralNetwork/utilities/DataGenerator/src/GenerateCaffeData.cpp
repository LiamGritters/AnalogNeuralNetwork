/*
 * GenerateCaffeData.cpp
 *
 *  Created on: 2019-02-18
 *      Author: liam
 */

/****************************************
 * INCLUDES
 ****************************************/

#include <gflags/gflags.h>
#include <boost/scoped_ptr.hpp>
#include <caffe/proto/caffe.pb.h>
#include <caffe/util/db.hpp>
#include <caffe/util/io.hpp>

#include <string>
#include <vector>
#include <algorithm>
#include <ctime>
#include <fstream>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

/****************************************
 * DEFINES
 ****************************************/

// define gflags FLAGS and default values
DEFINE_string(backend, "lmdb", "The backend {lmdb, leveldb} for storing the result");
DEFINE_int32(split, 1, "Number of samples {nr} used for TRAIN before a sample is used for TEST, use negative value to do the opposite");
DEFINE_bool(shuffle, true, "Randomly shuffle the order of samples");
DEFINE_bool(balance, false, "Create a balanced set");

/****************************************
 * Main
 ****************************************/

int
main(int argc, char* argv[])
{
    ::google::InitGoogleLogging(argv[0]);

#ifndef GFLAGS_GFLAGS_H_
    namespace gflags = google;
#endif

    gflags::SetUsageMessage("Generates random training data samples and puts it in\n"
        "the leveldb/lmdb format used as input for Caffe.\n"
        "Usage:\n"
        " generate-random-shape-training-data [FLAGS] DB_NAME\n");
    gflags::ParseCommandLineFlags(&argc, &argv, true);

    if (argc != 3)
    {
        gflags::ShowUsageWithFlagsRestrict(argv[0], "generate-random-shape-training-data");
        return 1;
    }

    // seed random generator
    std::srand(std::time(NULL));

    // generate random data
    typedef float tInput;
    typedef std::vector<tInput> tData;
    typedef int tLabel;
    typedef std::pair<tData, tLabel> tSample;
    typedef std::vector<tSample> tSamples;
    tSamples samples;

    //read in file
    std::ifstream inputFile;
    inputFile.open(argv[2]);
    if(inputFile.is_open())
    {
        std::string line;
        while(std::getline(inputFile, line))
        {
            std::stringstream lineStream(line);
            std::string cell;
            tData data;
            tLabel label;

            while(std::getline(lineStream, cell, ','))
            {
                data.push_back(std::atof(cell.c_str()));
            }

            label = (int)data[data.size()-1];
            data.erase(data.end()-1);
            samples.push_back(std::make_pair(data, label));
        }

        if(line.empty())
        {
            inputFile.close();
        }
    }

    // count classes and number of occurences
    typedef std::map<int, int> tCounts;
    tCounts counts;
    for (tSamples::const_iterator itrSample = samples.begin()
        ; itrSample != samples.end()
        ; ++itrSample)
    {
        counts[itrSample->second]++;
    }

    // show counts
    for (tCounts::const_iterator itr = counts.begin()
        ; itr != counts.end()
        ; ++itr)
    {
        std::cout << "class: " << itr->first << " count: " << itr->second << std::endl;
    }

    // shuffle the data
    if (FLAGS_shuffle == true)
    {
        // randomly shuffle samples
        std::random_shuffle(samples.begin(), samples.end());
    }

    // Create new train and test DB
    boost::scoped_ptr<caffe::db::DB> train_db(caffe::db::GetDB(FLAGS_backend));
    std::string dbTrainName = argv[1];
    dbTrainName += "_train";
    train_db->Open(dbTrainName.c_str(), caffe::db::NEW);
    boost::scoped_ptr<caffe::db::Transaction> train_txn(train_db->NewTransaction());

    boost::scoped_ptr<caffe::db::DB> test_db(caffe::db::GetDB(FLAGS_backend));
    std::string dbTestName = argv[1];
    dbTestName += "_test";
    test_db->Open(dbTestName.c_str(), caffe::db::NEW);
    boost::scoped_ptr<caffe::db::Transaction> test_txn(test_db->NewTransaction());

    // divide the train/test data, determine spliting tactic
    const int iSplitRate = FLAGS_split;
    int iNumberPutToTrain = 0;
    int iNumberPutToTest = 0;
    enum eNextSample { eNSTrain, eNSTest };
    eNextSample nextSample = eNSTrain; // always start with train samples

    // convert samples to caffe::Datum
    int iCount = 0, iCountTrain = 0, iCountTest = 0;
    for (tSamples::const_iterator itrSample = samples.begin()
        ; itrSample != samples.end()
        ; ++itrSample)
    {
        // extract label from sample
        const int iLabel = itrSample->second;

        // convert sample to protobuf Datum
        caffe::Datum datum;
        datum.set_channels(itrSample->first.size());
        datum.set_height(1);
        datum.set_width(1);
        datum.set_label(iLabel);
        for (tData::const_iterator itrInputData = itrSample->first.begin()
            ; itrInputData != itrSample->first.end()
            ; ++itrInputData)
        {
            datum.add_float_data(*itrInputData);
        }

        // write datum to db use the sample number as key for db
        std::string out;
        CHECK(datum.SerializeToString(&out));
        std::stringstream ss;
        ss << iCount;

        // put sample
        if (nextSample == eNSTrain) // always start with train samples
        {
            train_txn->Put(ss.str(), out);
            iNumberPutToTrain++;
            iCountTrain++;
        }
        else if (nextSample == eNSTest)
        {
            test_txn->Put(ss.str(), out);
            iNumberPutToTest++;
            iCountTest++;
        }

        // determine where next sample should go
        if (iSplitRate == 0)
        {
            nextSample = eNSTrain;
        }
        else if (iSplitRate < 0)
        {
            if (iNumberPutToTest == std::abs(iSplitRate))
            {
                nextSample = eNSTrain;
                iNumberPutToTest = 0;
            }
            else
            {
                nextSample = eNSTest;
            }
        }
        else if (iSplitRate > 0)
        {
            if (iNumberPutToTrain == iSplitRate)
            {
                nextSample = eNSTest;
                iNumberPutToTrain = 0;
            }
            else
            {
                nextSample = eNSTrain;
            }
        }

        // every 1000 samples commit to db
        if (iCountTrain % 1000 == 0)
        {
            train_txn->Commit();
            train_txn.reset(train_db->NewTransaction());
        }
        if (iCountTest % 1000 == 0)
        {
            test_txn->Commit();
            test_txn.reset(test_db->NewTransaction());
        }

        iCount++;
    }

    // commit the last unwritten batch
    if (iCountTrain % 1000 != 0)
    {
        train_txn->Commit();
    }
    if (iCountTest % 1000 != 0)
    {
        test_txn->Commit();
    }

    std::cout << "Total of " << iCount << " samples generated, put " << iCountTrain << " to TRAIN DB and " << iCountTest << " to TEST DB" << std::endl;
    return 0;
}
