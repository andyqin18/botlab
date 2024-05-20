#include <utils/grid_utils.hpp>
#include <slam/particle_filter.hpp>
#include <slam/occupancy_grid.hpp>
#include <utils/geometric/angle_functions.hpp>
#include <mbot_lcm_msgs/pose2D_t.hpp>
#include <mbot_lcm_msgs/particle_t.hpp>
#include <cassert>
#include <chrono>


ParticleFilter::ParticleFilter(int numParticles)
: kNumParticles_ (numParticles),
  samplingAugmentation_(0.5, 0.9, numParticles),
  distribution_quality(1),
  quality_reinvigoration_percentage(0.1)
{
    assert(kNumParticles_ > 1);
    posterior_.resize(kNumParticles_);
}


void ParticleFilter::initializeFilterAtPose(const mbot_lcm_msgs::pose2D_t& pose)
{
    ///////////// TODO: Implement your method for initializing the particles in the particle filter /////////////////
    double sampleWeight = 1.0 / kNumParticles_;
    posteriorPose_ = pose;

    for(auto& p : posterior_){
        p.pose.x = posteriorPose_.x;
        p.pose.y = posteriorPose_.y;
        p.pose.theta = wrap_to_pi(posteriorPose_.theta);
        p.pose.utime = pose.utime;
        p.parent_pose = p.pose;
        p.weight = sampleWeight;
    }
}

void ParticleFilter::initializeFilterRandomly(const OccupancyGrid& map)
{
    ///////////// TODO: Implement your method for initializing the particles in the particle filter /////////////////


}

void ParticleFilter::resetOdometry(const mbot_lcm_msgs::pose2D_t& odometry)
{
    actionModel_.resetPrevious(odometry);
}


mbot_lcm_msgs::pose2D_t ParticleFilter::updateFilter(const mbot_lcm_msgs::pose2D_t& odometry,
                                                        const mbot_lcm_msgs::lidar_t& laser,
                                                        const OccupancyGrid& map)
{
    const bool hasRobotMoved = actionModel_.updateAction(odometry);

    std::cerr << "[Debug] hasRobotMoved: " << hasRobotMoved << '\n';

    if (hasRobotMoved) {
        auto start = std::chrono::steady_clock::now();

        auto prior = resamplePosteriorDistribution(map); //check this- have done for nominal
        auto proposal = computeProposalDistribution(prior);
        posterior_ = computeNormalizedPosterior(proposal, laser, map);

        auto end = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration<double>(end - start);
        std::cout << "Particle filter runtime (" << kNumParticles_ << "): " << duration.count() << "\n";
    }

    /// TODO: Add reinvigoration step
    posteriorPose_ = estimatePosteriorPose(posterior_);
    posteriorPose_.utime = odometry.utime;

    return posteriorPose_;

    // updateFilterActionOnly(odometry);

    // sensorModel_

}

mbot_lcm_msgs::pose2D_t ParticleFilter::updateFilterActionOnly(const mbot_lcm_msgs::pose2D_t& odometry)
{
    // Only update the particles if motion was detected. If the robot didn't move, then
    // obviously don't do anything.
    const bool hasRobotMoved = actionModel_.updateAction(odometry);

    std::cerr << "[Debug] hasRobotMoved: " << hasRobotMoved << '\n';

    if(hasRobotMoved)
    {
        auto prior = resamplePosteriorDistribution();  // resample step
        auto proposal = computeProposalDistribution(prior);  // update
        posterior_ = proposal;
    }

    double running_max_likelihood = -std::numeric_limits<double>::infinity();
    for (const auto& particle : posterior_) {
        if (particle.weight > running_max_likelihood) {
            running_max_likelihood = particle.weight;
            posteriorPose_ = particle.pose;
        }
    }

    return posteriorPose_;
}


mbot_lcm_msgs::pose2D_t ParticleFilter::poseEstimate(void) const
{
    return posteriorPose_;
}


mbot_lcm_msgs::particles_t ParticleFilter::particles(void) const
{
    mbot_lcm_msgs::particles_t particles;
    particles.num_particles = posterior_.size();
    particles.particles = posterior_;
    return particles;
}


ParticleList ParticleFilter::resamplePosteriorDistribution(const OccupancyGrid& map,
                                                           const bool keep_best,
                                                           const bool reinvigorate)
{
    //////////// TODO: Implement your algorithm for resampling from the posterior distribution ///////////////////
    // ParticleList prior = posterior_;
    // double sampleWeight = 1.0/kNumParticles_;
    // std::random_device rd;
    // std::mt19937 generator(rd());
    // std::normal_distribution<> dist(0.0, 0.05);

    // for(auto& p: prior){
    //     p.pose.x = posteriorPose_.x + dist(generator);
    //     p.pose.y = posteriorPose_.y + dist(generator);
    //     p.pose.theta = posteriorPose_.theta + dist(generator);
    //     p.pose.utime = posteriorPose_.utime;
    //     p.parent_pose = posteriorPose_;
    //     p.weight = sampleWeight;
    // }


    // return prior;  // Placeholder
    return resamplePosteriorDistribution();
}


ParticleList ParticleFilter::resamplePosteriorDistribution(const bool keep_best,
                                                           const bool reinvigorate)
{
    //////////// TODO: Implement your algorithm for resampling from the posterior distribution ///////////////////
    std::vector<double> cumulative_weights(kNumParticles_);
    cumulative_weights[0] = posterior_[0].weight;
    for (size_t i = 1; i < posterior_.size(); ++i) {
        cumulative_weights[i] = cumulative_weights[i - 1] + posterior_[i].weight;
    }

    std::cerr << "[Debug] Resample: cumulative weight: " << cumulative_weights.back() << "\n";

    // Pick an initial random number and step through the
    // probability range with a fixed step size.
    const double step_size = cumulative_weights.back() / kNumParticles_;
    double sample_point = 0; // TODO: make this a random number

    // Resample by stepping forward the pointer
    ParticleList sampled_posterior;
    for (size_t i = 0; i < kNumParticles_; ++i) {
        while (cumulative_weights[i] > sample_point) {
            sampled_posterior.push_back(posterior_[i]);
            sample_point += step_size;
        }
    }

    return sampled_posterior;


    // double sampleWeight = 1.0/kNumParticles_;
    // std::random_device rd;
    // std::mt19937 generator(rd());
    // std::normal_distribution<> dist(0.0, 0.05);

    // for(auto& p: prior){
    //     p.pose.x = posteriorPose_.x + dist(generator);
    //     p.pose.y = posteriorPose_.y + dist(generator);
    //     p.pose.theta = posteriorPose_.theta + dist(generator);
    //     p.pose.utime = posteriorPose_.utime;
    //     p.parent_pose = posteriorPose_;
    //     p.weight = sampleWeight;
    // }


    // return prior;  // Placeholder
}


void ParticleFilter::reinvigoratePriorDistribution(ParticleList& prior)
{
    // Augmentation: if sensor model suspects an average particle quality of
    //      less than 15%, invigorate
    if (distribution_quality < 0.15)  // TODO: make 0.15 a parameter
    {
        int count = 0;
        int max_count = floor(quality_reinvigoration_percentage * prior.size());

        std::random_device rd;
        std::default_random_engine generator(rd());
        auto ud01 = std::uniform_real_distribution<double>(0.0, 1.0);
        int step = std::max<int>(1, floor(ud01(generator) * prior.size() / max_count));

        for (int i = 0; i < max_count; i++)
        {
            prior[i*step] = randomPoseGen_.get_particle();
        }

    }

    // // Augmentation: randomize any unreasonable samples
    // if(map != nullptr)
    // {
    //     for (int i = 0; i < prior.size(); i++)
    //     {
    //         const auto& p = prior[i].pose;
    //         if(!map->isCellInGrid(p.x, p.y) ||
    //           (map->isCellInGrid(p.x, p.y) && (map->logOdds(p.x, p.y) >= 0)))
    //         {
    //             std::cout << "\tinvalid sample!!" << ", "
    //                 << !map->isCellInGrid(p.x, p.y) << ", "
    //                 << (map->isCellInGrid(p.x, p.y) && (map->logOdds(p.x, p.y) >= 0.0))
    //                 << std::endl;


    //             prior[i] = randomPoseGen_.get_particle();
    //         }
    //     }
    // }
}


ParticleList ParticleFilter::computeProposalDistribution(const ParticleList& prior)
{
    //////////// TODO: Implement your algorithm for creating the proposal distribution by sampling from the ActionModel
    ParticleList proposal;
    for(auto & p:prior){
     proposal.push_back(actionModel_.applyAction(p));
    }

    return proposal;  // Placeholder
}


ParticleList ParticleFilter::computeNormalizedPosterior(const ParticleList& proposal,
                                                        const mbot_lcm_msgs::lidar_t& laser,
                                                        const OccupancyGrid& map)
{
    /////////// TODO: Implement your algorithm for computing the normalized posterior distribution using the
    ///////////       particles in the proposal distribution

    ParticleList posterior;
    double sumWeights = 0.0;
    for(auto& p:proposal){
       mbot_lcm_msgs::particle_t weighted = p;
       weighted.weight = sensorModel_.likelihood(weighted, laser, map);
       sumWeights += weighted.weight;
       posterior.push_back(weighted);
    }

    std::cerr << "[SENSOR MODEL DEBUG] sumWeights: " << sumWeights << "\n";

    for(auto& p:posterior){
        p.weight /= sumWeights;
    }

    return posterior;  // Placeholder
}


mbot_lcm_msgs::pose2D_t ParticleFilter::estimatePosteriorPose(const ParticleList& posterior)
{
    //////// TODO: Implement your method for computing the final pose estimate based on the posterior distribution
    // Figure out which pose to take for the posterior pose
    // Weighted average is simple, but could be very bad
    // Maybe only take the best x% and then average.


    mbot_lcm_msgs::pose2D_t pose;
    double xMean = 0.0;
    double yMean = 0.0;
    double cosThetaMean = 0.0;
    double sinThetaMean = 0.0;

    for(auto& p: posterior){
        xMean += p.weight*p.pose.x;
        yMean += p.weight*p.pose.y;
        cosThetaMean += p.weight*std::cos(p.pose.theta);
        sinThetaMean += p.weight*std::sin(p.pose.theta);
    }

    pose.x = xMean;
    pose.y = yMean;
    pose.theta = std::atan2(sinThetaMean,cosThetaMean);

    return pose;


}

mbot_lcm_msgs::pose2D_t ParticleFilter::computeParticlesAverage(const ParticleList& particles_to_average)
{
    mbot_lcm_msgs::pose2D_t avg_pose;
    avg_pose.x = 0.0;
    avg_pose.y = 0.0;
    avg_pose.theta = 0.0;
    double sum_weight = 0.0;

    // Aux variables to compute theta average
    double theta_x = 0.0;
    double theta_y = 0.0;
    for (auto &&p : particles_to_average)
    {
        avg_pose.x += p.weight * p.pose.x;
        avg_pose.y += p.weight * p.pose.y;
        theta_x += p.weight * std::cos(p.pose.theta);
        theta_y += p.weight * std::sin(p.pose.theta);

        sum_weight += p.weight;
    }
    avg_pose.x /= sum_weight;
    avg_pose.y /= sum_weight;
    theta_x /= sum_weight;
    theta_y /= sum_weight;
    avg_pose.theta = std::atan2(theta_y, theta_x);

    return avg_pose;
}
