#include "crow.h"

#include <hiredis/hiredis.h>
#include <iostream>
#include <string>

#include <sstream>

// g++ redis.cpp -I ../include -lhiredis  -lboost_system -lpthread -std=c++14
// #include <ros/ros.h>
// #include <ros/package.h>

class ExampleLogHandler : public crow::ILogHandler {
    public:
        void log(std::string /*message*/, crow::LogLevel /*level*/) override {
//            cerr << "ExampleLogHandler -> " << message;
        }
};

struct ExampleMiddleware 
{
    std::string message;

    ExampleMiddleware() 
    {
        message = "foo";
    }

    void setMessage(std::string newMsg)
    {
        message = newMsg;
    }

    struct context
    {
    };

    void before_handle(crow::request& /*req*/, crow::response& /*res*/, context& /*ctx*/)
    {
        CROW_LOG_DEBUG << " - MESSAGE: " << message;
    }

    void after_handle(crow::request& /*req*/, crow::response& /*res*/, context& /*ctx*/)
    {
        // no-op
    }
};


redisContext *pRedisContext = nullptr;
int main()
{

	struct timeval timeout = {2, 0};    //2s的超时时间
	//redisContext是Redis操作对象
	pRedisContext = (redisContext*)redisConnectWithTimeout("127.0.0.1", 6379, timeout);

	if ( (NULL == pRedisContext) || (pRedisContext->err) )
	{
		if (pRedisContext)
		{
			std::cout << "connect error:" << pRedisContext->errstr << std::endl;
		}
		else
		{
			std::cout << "connect error: can't allocate redis context." << std::endl;
		}
		return -1;
	}

    crow::App<ExampleMiddleware> app;

    app.get_middleware<ExampleMiddleware>().setMessage("redis");

    CROW_ROUTE(app, "/")
        .name("hello")
    ([]{
        return "Hello World!";
    });

    CROW_ROUTE(app, "/about")
    ([](){
        return "About Crow example.";
    });

    // a request to /path should be forwarded to /path/
    CROW_ROUTE(app, "/path/")
    ([](){
        return "Trailing slash test case..";
    });


    // simple json response
    // To see it in action enter {ip}:18080/json
    CROW_ROUTE(app, "/json")
    ([]{
        crow::json::wvalue x;
        x["message"] = "Hello, World!";
        return x;
    });

    // To see it in action enter {ip}:18080/hello/{integer_between -2^32 and 100} and you should receive
    // {integer_between -2^31 and 100} bottles of beer!
    CROW_ROUTE(app,"/hello/<int>")
    ([](int count){
        if (count > 100)
            return crow::response(400);
        std::ostringstream os;
        os << count << " bottles of beer!";
        return crow::response(os.str());
    });

    // To see it in action submit {ip}:18080/add/1/2 and you should receive 3 (exciting, isn't it)
    CROW_ROUTE(app,"/add/<int>/<int>")
    ([](const crow::request& /*req*/, crow::response& res, int a, int b){
        std::ostringstream os;
        os << a+b;
        res.write(os.str());
        res.end();
    });

    // Compile error with message "Handler type is mismatched with URL paramters"
    //CROW_ROUTE(app,"/another/<int>")
    //([](int a, int b){
        //return crow::response(500);
    //});

    // more json example

    // To see it in action, I recommend to use the Postman Chrome extension:
    //      * Set the address to {ip}:18080/add_json
    //      * Set the method to post
    //      * Select 'raw' and then JSON
    //      * Add {"a": 1, "b": 1}
    //      * Send and you should receive 2

    // A simpler way for json example:
    //      * curl -d '{"a":1,"b":2}' {ip}:18080/add_json
    CROW_ROUTE(app, "/add_json")
        .methods("POST"_method)
    ([](const crow::request& req){
        auto x = crow::json::load(req.body);
        if (!x)
            return crow::response(400);
        int sum = x["a"].i()+x["b"].i();
        std::ostringstream os;
        os << sum;
        return crow::response{os.str()};
    });

 	crow::mustache::set_base(".");

    CROW_ROUTE(app, "/data")
		([](){
		 return crow::mustache::load("data.tsv").render();
		 });

    CROW_ROUTE(app, "/json_wave")
		([](){

			// std::string filename(ros::package::getPath("arobot_db")+"/webs/");
		 	return crow::mustache::load("draw.html").render();
		 });

    CROW_ROUTE(app, "/json_get")
		.methods("POST"_method)
		([](const crow::request& req){

		 std::ostringstream os;


		 std::cout<< req.body <<std::endl;

		 auto x = req.body;
		 
		 x = "hi";

		 //redisReply是Redis命令回复对象 redis返回的信息保存在redisReply对象中
		 redisReply *pRedisReply = (redisReply*)redisCommand(pRedisContext, "LINDEX %s -1", x.c_str());  //Get the last id
		 //当多条Redis命令使用同一个redisReply对象时 
		 //每一次执行完Redis命令后需要清空redisReply 以免对下一次的Redis操作造成影响
		 os << pRedisReply->str;
		 std::cout<< pRedisReply->str <<std::endl;
		 freeReplyObject(pRedisReply);   

		// auto x = crow::json::load(req.body);
		// if (!x)
		//	 return crow::response(400);
		 // int sum = x["a"].i()+x["b"].i();
		 return crow::response{os.str()};
		 });
	  //var key = Request["hi"];

    // Example of a request taking URL parameters
    // If you want to activate all the functions just query
    // {ip}:18080/params?foo='blabla'&pew=32&count[]=a&count[]=b
    CROW_ROUTE(app, "/params")
    ([](const crow::request& req){
        std::ostringstream os;

        // To get a simple string from the url params
        // To see it in action /params?foo='blabla'
        os << "Params: " << req.url_params << "\n\n"; 
        os << "The key 'foo' was " << (req.url_params.get("foo") == nullptr ? "not " : "") << "found.\n";

        // To get a double from the request
        // To see in action submit something like '/params?pew=42'
        if(req.url_params.get("pew") != nullptr) {
            double countD = boost::lexical_cast<double>(req.url_params.get("pew"));
            os << "The value of 'pew' is " <<  countD << '\n';
        }

        // To get a list from the request
        // You have to submit something like '/params?count[]=a&count[]=b' to have a list with two values (a and b)
        auto count = req.url_params.get_list("count");
        os << "The key 'count' contains " << count.size() << " value(s).\n";
        for(const auto& countVal : count) {
            os << " - " << countVal << '\n';
        }
        return crow::response{os.str()};
    });    

    CROW_ROUTE(app, "/large")
    ([]{
        return std::string(512*1024, ' ');
    });

    // ignore all log
    crow::logger::setLogLevel(crow::LogLevel::DEBUG);
    //crow::logger::setHandler(std::make_shared<ExampleLogHandler>());

    app.port(18080)
        .multithreaded()
        .run();
}
