-module(socketcan).
-export([start/0, stop/0]).
-export([twice/1, sum/2, open/1, send/3]).

start() ->
  start("socketcan_driver").

start(SharedLib) ->
  case erl_ddll:load_driver("build", SharedLib) of
    ok -> ok;
    {error, already_loaded} -> ok;
    _ -> exit({error, could_not_load_driver})
  end,
  spawn(fun() -> init(SharedLib) end).

init(SharedLib) ->
  register(socketcan_lid, self()),
  Port = open_port({spawn, SharedLib}, []),
  loop(Port).

stop() ->
  socketcan_lid ! stop.

twice(X) -> call_port({twice, X}).
sum(X, Y) -> call_port({sum, X, Y}).
open(Port) -> call_port({open, Port}).
send(Socket, CanID, Data) -> call_port({send, Socket, CanID, Data}).

call_port(Msg) ->
  socketcan_lid ! {call, self(), Msg},
  receive
    {socketcan_lid, Result} ->
      Result
  end.

loop(Port) ->
  receive
    {call, Caller, Msg} ->
      Port ! {self(), {command, encode(Msg)}},
      receive
        {Port, {data, Data}} ->
          Caller ! {socketcan_lid, decode(Data)}
      end,
      loop(Port);
    stop ->
      Port ! {self(), close},
      receive
        {Port, closed} ->
          exit(normal)
      end;
    {'EXIT', Port, Reason} ->
      io:format("~p ~n", [Reason]),
      exit(port_terminated)
  end.

encode({twice, X}) -> [1, X];
encode({sum, X, Y}) -> [2, X, Y];
encode({open, Port}) -> [3, Port];
encode({send, Socket, CanID, Data}) -> [4, Socket, CanID, Data].

decode([Int]) -> Int.
