#ifndef RANDOM_NUMBER_GENERATOR
#define RANDOM_NUMBER_GENERATOR
#include <boost/random/normal_distribution.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/random/uniform_int.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/math/constants/constants.hpp>
#include <boost/random/lagged_fibonacci.hpp>
#include <boost/random/mersenne_twister.hpp>

namespace utils
{
	class RandomNumberGenerator
	{
	public:
		RandomNumberGenerator(void)
		: 	generator_(nextSeed()),
			uniDist_(0,1),
			normalDist_(0,1),
			uni_(generator_, uniDist_),
			normal_(generator_, normalDist_)

		{

		}

		double uniform01(void){
			return uni_();
		}

		double uniformReal(double lower_bound, double upper_bound){
			assert(lower_bound <= upper_bound);
			return (upper_bound - lower_bound) * uni_() + lower_bound;
		}

		int uniformInt(int lower_bound, int upper_bound){
			int r = (int)floor(uniformReal((double)lower_bound, (double)(upper_bound) + 1.0));
			return (r > upper_bound) ? upper_bound : r;
		}

		bool uniformBool(void){
			return uni_() <= 0.5;
		}

		double gaussian01(void){
			return normal_();
		}

		double gaussian(double mean, double stddev){
			return normal_() * stddev + mean;
		}


		static void setSeed(boost::uint32_t seed){
		    if (getFirstSeedGenerated())
		    {
		        return;
		    }
		    if (seed == 0)
		    {
		        getUserSetSeed() = 1;
		    }
		    else
		        getUserSetSeed() = seed;
		}

		static boost::uint32_t getSeed(void){
			return firstSeed();
		}

	private:
		static boost::uint32_t nextSeed(void){
			static boost::mutex rngMutex;
			boost::mutex::scoped_lock slock(rngMutex);
			static boost::lagged_fibonacci607 sGen(firstSeed());
			static boost::uniform_int<> sDist(1,1000000000);
			static boost::variate_generator<boost::lagged_fibonacci607, boost::uniform_int<> > s(sGen, sDist);
			return s();
		}

		static boost::uint32_t firstSeed(void){
			static boost::uint32_t firstSeedValue = 0;
			static boost::mutex fsLock;
			boost::mutex::scoped_lock slock(fsLock);
			if(getFirstSeedGenerated())
				return firstSeedValue;

			if(getUserSetSeed() != 0)
				firstSeedValue = getUserSetSeed();
			else
				firstSeedValue = (boost::uint32_t) (boost::posix_time::microsec_clock::universal_time() -
                              boost::posix_time::ptime(boost::date_time::min_date_time)).total_microseconds();

			getFirstSeedGenerated() = true;

			return firstSeedValue;
		}

		static bool& getFirstSeedGenerated(void)
		{
		    static bool firstSeedGenerated = false;
		    return firstSeedGenerated;
		}

		static boost::uint32_t& getUserSetSeed(void)
		{
		    static boost::uint32_t userSetSeed = 0;
		    return userSetSeed;
		}


		boost::mt19937 generator_;
		boost::uniform_real<> uniDist_;
		boost::normal_distribution<> normalDist_;
		boost::variate_generator<boost::mt19937&, boost::uniform_real<> > uni_;
		boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > normal_;
	};
}


#endif